use bsw_ethernet::{
    ConfigRegistry, InterfaceBackend, InterfaceState, NetworkBackendError, NetworkConfig,
    NetworkInterface,
};
use bsw_lifecycle::{LifecycleComponent, TransitionResult};
use bsw_logger::{ComponentId, Level, Log};

#[derive(Default)]
struct CaptureLog {
    entries: Vec<(Level, String)>,
}

impl Log for CaptureLog {
    fn enabled(&self, _: ComponentId, _: Level) -> bool {
        true
    }

    fn log(&mut self, _: ComponentId, level: Level, args: core::fmt::Arguments<'_>) {
        self.entries.push((level, args.to_string()));
    }
}

#[derive(Default)]
struct HostBackend {
    initialized: bool,
    started: bool,
    fail_start_once: bool,
    dynamic: Option<NetworkConfig>,
    calls: Vec<&'static str>,
}

impl InterfaceBackend for HostBackend {
    fn initialize(&mut self, _: NetworkConfig) -> Result<(), NetworkBackendError> {
        self.calls.push("initialize");
        self.initialized = true;
        Ok(())
    }

    fn start(&mut self) -> Result<(), NetworkBackendError> {
        self.calls.push("start");
        if self.fail_start_once {
            self.fail_start_once = false;
            return Err(NetworkBackendError::StartFailed);
        }
        if !self.initialized {
            return Err(NetworkBackendError::StartFailed);
        }
        self.started = true;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), NetworkBackendError> {
        self.calls.push("stop");
        self.initialized = false;
        self.started = false;
        Ok(())
    }

    fn poll_address_change(&mut self) -> Option<NetworkConfig> {
        self.dynamic.take()
    }
}

fn config(last: u8) -> NetworkConfig {
    NetworkConfig::ipv4([198, 51, 100, last], [255, 255, 255, 0], [198, 51, 100, 1])
}

#[test]
fn lifecycle_restart_and_address_change_recover_cleanly() {
    let backend = HostBackend::default();
    let logger = CaptureLog::default();
    let mut interface = NetworkInterface::new("eth-test", 7, config(20), backend, logger);

    assert_eq!(interface.init(), TransitionResult::Done);
    assert_eq!(interface.run(), TransitionResult::Done);
    assert_eq!(interface.state(), InterfaceState::Started);

    assert_eq!(interface.reconfigure(config(21)), Ok(true));
    assert_eq!(interface.state(), InterfaceState::Started);
    assert_eq!(interface.active_config(), config(21));
    assert_eq!(interface.restart_count(), 1);
    assert_eq!(
        interface.backend().calls,
        ["initialize", "start", "stop", "initialize", "start"]
    );

    assert_eq!(interface.shutdown(), TransitionResult::Done);
    assert_eq!(interface.shutdown(), TransitionResult::Done);
    assert_eq!(interface.state(), InterfaceState::Uninitialized);
    assert_eq!(interface.init(), TransitionResult::Done);
    assert_eq!(interface.run(), TransitionResult::Done);
    assert_eq!(interface.state(), InterfaceState::Started);
}

#[test]
fn dynamic_address_fault_can_be_retried_without_leaking_state() {
    let backend = HostBackend {
        dynamic: Some(config(31)),
        ..HostBackend::default()
    };
    let mut interface =
        NetworkInterface::new("eth-test", 4, config(30), backend, CaptureLog::default());
    assert_eq!(interface.init(), TransitionResult::Done);
    assert_eq!(interface.run(), TransitionResult::Done);
    interface.backend_mut().fail_start_once = true;

    assert_eq!(interface.cycle(), Err(NetworkBackendError::StartFailed));
    assert_eq!(
        interface.state(),
        InterfaceState::Faulted(NetworkBackendError::StartFailed)
    );
    assert_eq!(interface.active_config(), NetworkConfig::Invalid);

    assert_eq!(interface.init(), TransitionResult::Done);
    assert_eq!(interface.run(), TransitionResult::Done);
    assert_eq!(interface.active_config(), config(31));
}

#[test]
fn registry_matches_upstream_change_semantics_and_bounds_events() {
    let mut registry: ConfigRegistry<2, 1> =
        ConfigRegistry::new([4, 7], [NetworkConfig::Invalid, config(40)]);
    assert_eq!(registry.get(99), NetworkConfig::Invalid);
    assert!(!registry.update(99, config(41)));
    assert!(!registry.update(7, config(40)));
    assert!(registry.update(4, config(42)));
    assert!(registry.update(7, config(43)));
    assert_eq!(registry.dropped_changes(), 1);
    let change = registry.take_change().unwrap();
    assert_eq!(change.bus_id, 4);
    assert_eq!(change.config, config(42));
    assert_eq!(registry.take_change(), None);
}

#[test]
fn invalid_configuration_fails_before_backend_initialization() {
    let mut interface = NetworkInterface::new(
        "eth-test",
        1,
        NetworkConfig::Invalid,
        HostBackend::default(),
        CaptureLog::default(),
    );
    assert_eq!(interface.init(), TransitionResult::Error);
    assert_eq!(
        interface.state(),
        InterfaceState::Faulted(NetworkBackendError::InvalidConfiguration)
    );
    assert!(interface.backend().calls.is_empty());
}
