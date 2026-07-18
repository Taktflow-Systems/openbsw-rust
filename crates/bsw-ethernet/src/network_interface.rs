//! Network-interface configuration and lifecycle integration (package D21).
//!
//! This is the allocation-free Rust boundary for upstream
//! `lwipSocket/netif/LwipNetworkInterface` and
//! `ip::NetworkInterfaceConfigRegistry`. A platform backend performs the
//! actual interface operations while this module owns state ordering,
//! configuration-change publication, restart recovery, and logging.

use bsw_lifecycle::{LifecycleComponent, TransitionResult};
use bsw_logger::{bsw_log, ComponentId, Level, Log};

use crate::NetworkConfig;

/// Logger component reserved for Ethernet interface lifecycle messages.
pub const NETWORK_LOG_COMPONENT: ComponentId = ComponentId::new(0);

/// Platform-independent network backend error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BackendError {
    /// The requested configuration is invalid or unsupported.
    InvalidConfiguration,
    /// The backend could not allocate or initialize interface resources.
    InitializationFailed,
    /// The backend could not bring the interface up.
    StartFailed,
    /// The backend could not shut the interface down cleanly.
    StopFailed,
}

/// Operations required from an lwIP, POSIX, or simulated interface backend.
///
/// Implementations must make `stop` idempotent. After a successful `stop`, a
/// later `initialize`/`start` pair must be supported so address changes and
/// lifecycle restarts do not leak resources.
pub trait InterfaceBackend {
    /// Allocate/configure the interface without making it operational.
    fn initialize(&mut self, config: NetworkConfig) -> Result<(), BackendError>;

    /// Bring the initialized interface up.
    fn start(&mut self) -> Result<(), BackendError>;

    /// Bring the interface down and release resources.
    fn stop(&mut self) -> Result<(), BackendError>;

    /// Report an address assigned asynchronously (for example by DHCP).
    fn poll_address_change(&mut self) -> Option<NetworkConfig> {
        None
    }
}

/// Settled state of a [`NetworkInterface`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterfaceState {
    /// No backend resources are owned.
    Uninitialized,
    /// Backend resources exist but the interface is down.
    Initialized,
    /// Interface is operational.
    Started,
    /// A backend operation failed. Shutdown/re-initialization remains legal.
    Faulted(BackendError),
}

/// One configuration-change notification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigChange {
    /// Application-defined network bus identifier.
    pub bus_id: u8,
    /// Newly active configuration, or [`NetworkConfig::Invalid`] when down.
    pub config: NetworkConfig,
}

/// Fixed-capacity network configuration registry.
///
/// Upstream uses an `etl::signal`; the Rust port exposes the same ordered,
/// bounded observation through an event queue. Every successful change is
/// visible exactly once, unchanged values produce no event, and queue
/// overflow is counted rather than allocating.
pub struct ConfigRegistry<const N: usize, const E: usize> {
    bus_ids: [u8; N],
    configs: [NetworkConfig; N],
    events: [Option<ConfigChange>; E],
    event_head: usize,
    event_len: usize,
    dropped: u32,
}

impl<const N: usize, const E: usize> ConfigRegistry<N, E> {
    /// Create a registry with one initial value per bus.
    pub const fn new(bus_ids: [u8; N], configs: [NetworkConfig; N]) -> Self {
        Self {
            bus_ids,
            configs,
            events: [None; E],
            event_head: 0,
            event_len: 0,
            dropped: 0,
        }
    }

    /// Return a bus configuration, or invalid for an unknown bus.
    pub fn get(&self, bus_id: u8) -> NetworkConfig {
        self.bus_ids
            .iter()
            .position(|candidate| *candidate == bus_id)
            .map_or(NetworkConfig::Invalid, |index| self.configs[index])
    }

    /// Update one known bus. Returns whether the value changed.
    pub fn update(&mut self, bus_id: u8, config: NetworkConfig) -> bool {
        let Some(index) = self
            .bus_ids
            .iter()
            .position(|candidate| *candidate == bus_id)
        else {
            return false;
        };
        if self.configs[index] == config {
            return false;
        }
        self.configs[index] = config;
        self.push(ConfigChange { bus_id, config });
        true
    }

    /// Pop the oldest pending configuration change.
    pub fn take_change(&mut self) -> Option<ConfigChange> {
        if self.event_len == 0 || E == 0 {
            return None;
        }
        let event = self.events[self.event_head].take();
        self.event_head = (self.event_head + 1) % E;
        self.event_len -= 1;
        event
    }

    /// Events dropped because the fixed queue was full.
    pub const fn dropped_changes(&self) -> u32 {
        self.dropped
    }

    fn push(&mut self, event: ConfigChange) {
        if E == 0 || self.event_len == E {
            self.dropped = self.dropped.saturating_add(1);
            return;
        }
        self.events[(self.event_head + self.event_len) % E] = Some(event);
        self.event_len += 1;
    }
}

/// Lifecycle-owned network interface with restart-safe configuration changes.
pub struct NetworkInterface<B, L> {
    name: &'static str,
    bus_id: u8,
    configured: NetworkConfig,
    active: NetworkConfig,
    state: InterfaceState,
    backend: B,
    logger: L,
    restart_count: u32,
}

impl<B: InterfaceBackend, L: Log> NetworkInterface<B, L> {
    /// Construct an interface in the uninitialized state.
    pub const fn new(
        name: &'static str,
        bus_id: u8,
        configured: NetworkConfig,
        backend: B,
        logger: L,
    ) -> Self {
        Self {
            name,
            bus_id,
            configured,
            active: NetworkConfig::Invalid,
            state: InterfaceState::Uninitialized,
            backend,
            logger,
            restart_count: 0,
        }
    }

    /// Current interface state.
    pub const fn state(&self) -> InterfaceState {
        self.state
    }

    /// Configuration currently reported to consumers.
    pub const fn active_config(&self) -> NetworkConfig {
        self.active
    }

    /// Number of successful running address-change restarts.
    pub const fn restart_count(&self) -> u32 {
        self.restart_count
    }

    /// Access the backend for platform-specific inspection or fault injection.
    pub const fn backend(&self) -> &B {
        &self.backend
    }

    /// Mutable backend access for platform-specific test controls.
    pub fn backend_mut(&mut self) -> &mut B {
        &mut self.backend
    }

    /// Change the desired address configuration.
    ///
    /// When running, the old instance is stopped before the new address is
    /// initialized. A failed restart leaves the interface faulted and with an
    /// invalid active configuration; a later call can recover it.
    pub fn reconfigure(&mut self, config: NetworkConfig) -> Result<bool, BackendError> {
        if self.configured == config && self.active == config {
            return Ok(false);
        }
        self.configured = config;
        if self.state != InterfaceState::Started {
            return Ok(true);
        }

        bsw_log!(
            &mut self.logger,
            NETWORK_LOG_COMPONENT,
            Level::Info,
            "network interface {} restarting after address change",
            self.name
        );
        self.stop_backend()?;
        self.initialize_backend()?;
        self.start_backend()?;
        self.restart_count = self.restart_count.saturating_add(1);
        Ok(true)
    }

    /// Poll backend-owned dynamic address assignment and apply a change.
    pub fn cycle(&mut self) -> Result<bool, BackendError> {
        let Some(config) = self.backend.poll_address_change() else {
            return Ok(false);
        };
        if self.active == config {
            return Ok(false);
        }
        self.reconfigure(config)
    }

    /// Publish the current value into a registry.
    pub fn publish<const N: usize, const E: usize>(
        &self,
        registry: &mut ConfigRegistry<N, E>,
    ) -> bool {
        registry.update(self.bus_id, self.active)
    }

    fn initialize_backend(&mut self) -> Result<(), BackendError> {
        if !self.configured.is_valid() {
            return self.fail(BackendError::InvalidConfiguration);
        }
        if let Err(error) = self.backend.initialize(self.configured) {
            return self.fail(error);
        }
        self.state = InterfaceState::Initialized;
        bsw_log!(
            &mut self.logger,
            NETWORK_LOG_COMPONENT,
            Level::Info,
            "network interface {} initialized",
            self.name
        );
        Ok(())
    }

    fn start_backend(&mut self) -> Result<(), BackendError> {
        if let Err(error) = self.backend.start() {
            return self.fail(error);
        }
        self.active = self.configured;
        self.state = InterfaceState::Started;
        bsw_log!(
            &mut self.logger,
            NETWORK_LOG_COMPONENT,
            Level::Info,
            "network interface {} started",
            self.name
        );
        Ok(())
    }

    fn stop_backend(&mut self) -> Result<(), BackendError> {
        if self.state == InterfaceState::Uninitialized {
            self.active = NetworkConfig::Invalid;
            return Ok(());
        }
        if let Err(error) = self.backend.stop() {
            return self.fail(error);
        }
        self.active = NetworkConfig::Invalid;
        self.state = InterfaceState::Uninitialized;
        bsw_log!(
            &mut self.logger,
            NETWORK_LOG_COMPONENT,
            Level::Info,
            "network interface {} stopped",
            self.name
        );
        Ok(())
    }

    fn fail<T>(&mut self, error: BackendError) -> Result<T, BackendError> {
        self.active = NetworkConfig::Invalid;
        self.state = InterfaceState::Faulted(error);
        bsw_log!(
            &mut self.logger,
            NETWORK_LOG_COMPONENT,
            Level::Error,
            "network interface {} operation failed",
            self.name
        );
        Err(error)
    }
}

impl<B: InterfaceBackend, L: Log> LifecycleComponent for NetworkInterface<B, L> {
    fn init(&mut self) -> TransitionResult {
        match self.state {
            InterfaceState::Initialized | InterfaceState::Started => TransitionResult::Done,
            InterfaceState::Uninitialized | InterfaceState::Faulted(_) => self
                .initialize_backend()
                .map_or(TransitionResult::Error, |()| TransitionResult::Done),
        }
    }

    fn run(&mut self) -> TransitionResult {
        match self.state {
            InterfaceState::Started => TransitionResult::Done,
            InterfaceState::Initialized => self
                .start_backend()
                .map_or(TransitionResult::Error, |()| TransitionResult::Done),
            InterfaceState::Uninitialized | InterfaceState::Faulted(_) => TransitionResult::Error,
        }
    }

    fn shutdown(&mut self) -> TransitionResult {
        self.stop_backend()
            .map_or(TransitionResult::Error, |()| TransitionResult::Done)
    }

    fn name(&self) -> &str {
        self.name
    }
}
