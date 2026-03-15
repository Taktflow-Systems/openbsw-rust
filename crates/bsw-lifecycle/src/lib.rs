// Copyright 2024 Accenture / Taktflow Systems.
//
// Rust port of OpenBSW `lifecycle`.
//
// Design:
//   - `#![no_std]` compatible — uses `core` only, zero heap allocation.
//   - Components are identified by their registration index.
//   - The manager tracks per-component state in a fixed-size inline array.
//   - `init_all`, `run_all`, `shutdown_all` operate on externally-owned
//     component slices, keeping the manager free of reference lifetime issues.

//! # bsw-lifecycle
//!
//! Component lifecycle management for embedded BSW — Rust port of OpenBSW `lifecycle`.
//!
//! The lifecycle system coordinates component startup, normal operation, and
//! orderly shutdown.  Each component implements [`LifecycleComponent`] and
//! registers with a [`LifecycleManager`].  The manager drives all registered
//! components through a defined sequence of states:
//!
//! ```text
//! Uninitialized
//!      │  init_component / init_all
//!      ▼
//! Transitioning(Init)
//!      │  Done / Error → Initialized   Pending → stays Transitioning
//!      ▼
//! Initialized
//!      │  run_component / run_all
//!      ▼
//! Transitioning(Run)
//!      │  Done / Error → Running       Pending → stays Transitioning
//!      ▼
//! Running
//!      │  shutdown_component / shutdown_all
//!      ▼
//! Transitioning(Shutdown)
//!      │  Done / Error → ShutDown      Pending → stays Transitioning
//!      ▼
//! ShutDown
//! ```
//!
//! ## Usage example
//!
//! ```rust
//! use bsw_lifecycle::{
//!     LifecycleComponent, LifecycleManager, ManagerState, TransitionResult,
//! };
//!
//! struct MyComponent;
//!
//! impl LifecycleComponent for MyComponent {
//!     fn init(&mut self)     -> TransitionResult { TransitionResult::Done }
//!     fn run(&mut self)      -> TransitionResult { TransitionResult::Done }
//!     fn shutdown(&mut self) -> TransitionResult { TransitionResult::Done }
//!     fn name(&self)         -> &str             { "MyComponent" }
//! }
//!
//! let mut mgr: LifecycleManager<4> = LifecycleManager::new();
//! let idx = mgr.register().expect("capacity not exhausted");
//! assert_eq!(idx, 0);
//!
//! let mut comp = MyComponent;
//! let mut components: [&mut dyn LifecycleComponent; 1] = [&mut comp];
//!
//! mgr.init_all(&mut components);
//! mgr.run_all(&mut components);
//! mgr.shutdown_all(&mut components);
//!
//! assert_eq!(mgr.state(), ManagerState::ShutDown);
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

// ---------------------------------------------------------------------------
// TransitionType
// ---------------------------------------------------------------------------

/// The kind of lifecycle transition currently in progress.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransitionType {
    /// System startup: initialize the component.
    Init,
    /// Normal operation: start the component running.
    Run,
    /// System shutdown: stop the component.
    Shutdown,
}

// ---------------------------------------------------------------------------
// ComponentState
// ---------------------------------------------------------------------------

/// The lifecycle state of a single registered component.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComponentState {
    /// Component has been registered but `init` has not been called yet.
    Uninitialized,
    /// A transition is in progress (async/deferred completion).
    Transitioning(TransitionType),
    /// `init` completed successfully; waiting for `run`.
    Initialized,
    /// `run` completed successfully; component is operating normally.
    Running,
    /// `shutdown` completed; component is quiescent.
    ShutDown,
}

// ---------------------------------------------------------------------------
// TransitionResult
// ---------------------------------------------------------------------------

/// Result returned by a [`LifecycleComponent`] method.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransitionResult {
    /// Transition completed synchronously.  The manager advances the
    /// component state immediately.
    Done,
    /// Transition started but is not yet complete (e.g., hardware not ready).
    /// The manager keeps the component in [`ComponentState::Transitioning`]
    /// until [`LifecycleManager::complete_transition`] is called.
    Pending,
    /// Transition encountered an error.  The component state is NOT advanced.
    Error,
}

// ---------------------------------------------------------------------------
// LifecycleComponent
// ---------------------------------------------------------------------------

/// Trait implemented by every BSW component that participates in lifecycle
/// management.
///
/// The manager calls these methods at the appropriate times.  Each method
/// returns a [`TransitionResult`] to indicate whether the transition
/// completed synchronously, is pending, or failed.
pub trait LifecycleComponent {
    /// Called during the Init phase.  Perform hardware setup, buffer
    /// initialisation, and similar one-time startup work here.
    fn init(&mut self) -> TransitionResult;

    /// Called during the Run phase.  Start background tasks, enable
    /// interrupts, begin periodic processing, etc.
    fn run(&mut self) -> TransitionResult;

    /// Called during the Shutdown phase.  Flush buffers, disable hardware,
    /// and release resources in an orderly manner.
    fn shutdown(&mut self) -> TransitionResult;

    /// Human-readable component name used in diagnostics and logging.
    fn name(&self) -> &str;
}

// ---------------------------------------------------------------------------
// ManagerState
// ---------------------------------------------------------------------------

/// Overall state of the [`LifecycleManager`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ManagerState {
    /// No transitions have been started yet.
    Idle,
    /// `init_all` has been called; components are being initialised.
    InitInProgress,
    /// All components finished Init and Run; system is operational.
    Running,
    /// `shutdown_all` has been called; components are being shut down.
    ShutdownInProgress,
    /// All components have completed Shutdown.
    ShutDown,
}

// ---------------------------------------------------------------------------
// LifecycleManager
// ---------------------------------------------------------------------------

/// Manages lifecycle transitions for up to `N` registered components.
///
/// Components do not need to be stored inside the manager.  The application
/// owns the component objects (typically a `static mut` array or a local
/// array in `main`).  The manager only tracks which index slot is allocated
/// and the current [`ComponentState`] of each slot.
///
/// # Const parameter
///
/// - `N` — maximum number of components that can be registered.
///
/// # Lifecycle sequence
///
/// 1. Register all components with [`register`](LifecycleManager::register).
/// 2. Call [`init_all`](LifecycleManager::init_all) to run the Init phase.
/// 3. Call [`run_all`](LifecycleManager::run_all) to run the Run phase.
/// 4. Call [`shutdown_all`](LifecycleManager::shutdown_all) to shut down.
///
/// For asynchronous transitions, each component that returns
/// [`TransitionResult::Pending`] must be completed later with
/// [`complete_transition`](LifecycleManager::complete_transition).
pub struct LifecycleManager<const N: usize> {
    states: [ComponentState; N],
    count: usize,
    manager_state: ManagerState,
}

impl<const N: usize> LifecycleManager<N> {
    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    /// Creates a new, empty lifecycle manager.
    ///
    /// All `N` component slots start as [`ComponentState::Uninitialized`] and
    /// are not yet registered.  The manager begins in [`ManagerState::Idle`].
    ///
    /// This function is `const` so it can be used to initialise statics.
    pub const fn new() -> Self {
        Self {
            states: [ComponentState::Uninitialized; N],
            count: 0,
            manager_state: ManagerState::Idle,
        }
    }

    // -----------------------------------------------------------------------
    // Registration
    // -----------------------------------------------------------------------

    /// Registers one component slot and returns its index.
    ///
    /// Returns `None` when the manager is already full (`N` components have
    /// been registered).  The returned index must be used with
    /// [`init_component`](LifecycleManager::init_component),
    /// [`component_state`](LifecycleManager::component_state), and
    /// [`complete_transition`](LifecycleManager::complete_transition).
    pub fn register(&mut self) -> Option<usize> {
        if self.count >= N {
            return None;
        }
        let idx = self.count;
        self.states[idx] = ComponentState::Uninitialized;
        self.count += 1;
        Some(idx)
    }

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    /// Returns the current [`ManagerState`].
    #[inline]
    pub fn state(&self) -> ManagerState {
        self.manager_state
    }

    /// Returns the current [`ComponentState`] for the component at `index`.
    ///
    /// Returns [`ComponentState::Uninitialized`] for any index beyond the
    /// registered count.
    #[inline]
    pub fn component_state(&self, index: usize) -> ComponentState {
        if index < self.count {
            self.states[index]
        } else {
            ComponentState::Uninitialized
        }
    }

    /// Returns the number of registered components.
    #[inline]
    pub fn count(&self) -> usize {
        self.count
    }

    // -----------------------------------------------------------------------
    // Single-component transitions
    // -----------------------------------------------------------------------

    /// Runs the `init` transition on the component at `index`.
    ///
    /// The component slice must contain at least `index + 1` entries, and
    /// `components[index]` must be the component that was registered at that
    /// index.
    ///
    /// State transitions:
    /// - [`TransitionResult::Done`]    → state becomes [`ComponentState::Initialized`]
    /// - [`TransitionResult::Pending`] → state becomes [`ComponentState::Transitioning`]`(Init)`
    /// - [`TransitionResult::Error`]   → state is unchanged
    ///
    /// Returns the result of calling `component.init()`.
    pub fn init_component(
        &mut self,
        index: usize,
        component: &mut dyn LifecycleComponent,
    ) -> TransitionResult {
        if index >= self.count {
            return TransitionResult::Error;
        }
        let result = component.init();
        match result {
            TransitionResult::Done => {
                self.states[index] = ComponentState::Initialized;
            }
            TransitionResult::Pending => {
                self.states[index] = ComponentState::Transitioning(TransitionType::Init);
            }
            TransitionResult::Error => {
                // state unchanged
            }
        }
        result
    }

    /// Runs the `run` transition on the component at `index`.
    ///
    /// State transitions:
    /// - [`TransitionResult::Done`]    → state becomes [`ComponentState::Running`]
    /// - [`TransitionResult::Pending`] → state becomes [`ComponentState::Transitioning`]`(Run)`
    /// - [`TransitionResult::Error`]   → state is unchanged
    pub fn run_component(
        &mut self,
        index: usize,
        component: &mut dyn LifecycleComponent,
    ) -> TransitionResult {
        if index >= self.count {
            return TransitionResult::Error;
        }
        let result = component.run();
        match result {
            TransitionResult::Done => {
                self.states[index] = ComponentState::Running;
            }
            TransitionResult::Pending => {
                self.states[index] = ComponentState::Transitioning(TransitionType::Run);
            }
            TransitionResult::Error => {
                // state unchanged
            }
        }
        result
    }

    /// Runs the `shutdown` transition on the component at `index`.
    ///
    /// State transitions:
    /// - [`TransitionResult::Done`]    → state becomes [`ComponentState::ShutDown`]
    /// - [`TransitionResult::Pending`] → state becomes [`ComponentState::Transitioning`]`(Shutdown)`
    /// - [`TransitionResult::Error`]   → state is unchanged
    pub fn shutdown_component(
        &mut self,
        index: usize,
        component: &mut dyn LifecycleComponent,
    ) -> TransitionResult {
        if index >= self.count {
            return TransitionResult::Error;
        }
        let result = component.shutdown();
        match result {
            TransitionResult::Done => {
                self.states[index] = ComponentState::ShutDown;
            }
            TransitionResult::Pending => {
                self.states[index] = ComponentState::Transitioning(TransitionType::Shutdown);
            }
            TransitionResult::Error => {
                // state unchanged
            }
        }
        result
    }

    // -----------------------------------------------------------------------
    // Batch transitions
    // -----------------------------------------------------------------------

    /// Runs `init` on every registered component in the `components` slice.
    ///
    /// - The slice must contain exactly [`count`](LifecycleManager::count)
    ///   entries in registration order.
    /// - Only components in [`ComponentState::Uninitialized`] are transitioned;
    ///   all others are skipped silently.
    /// - The manager state is set to [`ManagerState::InitInProgress`].
    ///
    /// Returns the number of components whose transition is still
    /// [`TransitionResult::Pending`] (i.e., not yet done).
    pub fn init_all(&mut self, components: &mut [&mut dyn LifecycleComponent]) -> usize {
        self.manager_state = ManagerState::InitInProgress;
        let n = self.count.min(components.len());
        let mut pending = 0usize;
        for (i, component) in components[..n].iter_mut().enumerate() {
            if self.states[i] == ComponentState::Uninitialized {
                let result = component.init();
                match result {
                    TransitionResult::Done => {
                        self.states[i] = ComponentState::Initialized;
                    }
                    TransitionResult::Pending => {
                        self.states[i] =
                            ComponentState::Transitioning(TransitionType::Init);
                        pending += 1;
                    }
                    TransitionResult::Error => {
                        // state unchanged
                    }
                }
            }
        }
        pending
    }

    /// Runs `run` on every component in [`ComponentState::Initialized`].
    ///
    /// - The slice must contain exactly [`count`](LifecycleManager::count)
    ///   entries in registration order.
    /// - The manager state is set to [`ManagerState::Running`] before
    ///   iterating so that pending completions during the loop see the correct
    ///   manager state.
    ///
    /// Returns the number of components whose transition is still pending.
    pub fn run_all(&mut self, components: &mut [&mut dyn LifecycleComponent]) -> usize {
        self.manager_state = ManagerState::Running;
        let n = self.count.min(components.len());
        let mut pending = 0usize;
        for (i, component) in components[..n].iter_mut().enumerate() {
            if self.states[i] == ComponentState::Initialized {
                let result = component.run();
                match result {
                    TransitionResult::Done => {
                        self.states[i] = ComponentState::Running;
                    }
                    TransitionResult::Pending => {
                        self.states[i] =
                            ComponentState::Transitioning(TransitionType::Run);
                        pending += 1;
                    }
                    TransitionResult::Error => {
                        // state unchanged
                    }
                }
            }
        }
        pending
    }

    /// Runs `shutdown` on every component in [`ComponentState::Running`], in
    /// **reverse** registration order.
    ///
    /// - The slice must contain exactly [`count`](LifecycleManager::count)
    ///   entries in registration order.
    /// - The manager state is set to [`ManagerState::ShutdownInProgress`].
    /// - When no components are pending after processing, the manager state
    ///   advances to [`ManagerState::ShutDown`].
    ///
    /// Returns the number of components whose transition is still pending.
    pub fn shutdown_all(&mut self, components: &mut [&mut dyn LifecycleComponent]) -> usize {
        self.manager_state = ManagerState::ShutdownInProgress;
        let n = self.count.min(components.len());
        let mut pending = 0usize;
        // Reverse order: last-registered component shuts down first.
        for i in (0..n).rev() {
            if self.states[i] == ComponentState::Running {
                let result = components[i].shutdown();
                match result {
                    TransitionResult::Done => {
                        self.states[i] = ComponentState::ShutDown;
                    }
                    TransitionResult::Pending => {
                        self.states[i] =
                            ComponentState::Transitioning(TransitionType::Shutdown);
                        pending += 1;
                    }
                    TransitionResult::Error => {
                        // state unchanged
                    }
                }
            }
        }
        if pending == 0 {
            self.manager_state = ManagerState::ShutDown;
        }
        pending
    }

    // -----------------------------------------------------------------------
    // Async completion
    // -----------------------------------------------------------------------

    /// Marks the pending transition for the component at `index` as complete.
    ///
    /// The component must be in [`ComponentState::Transitioning`].  The state
    /// is advanced to the post-transition state for the transition type:
    ///
    /// | Transition in progress | New state after completion |
    /// |------------------------|---------------------------|
    /// | `Init`                 | `Initialized`             |
    /// | `Run`                  | `Running`                 |
    /// | `Shutdown`             | `ShutDown`                |
    ///
    /// If the component is not in `Transitioning` state, this call is a no-op.
    pub fn complete_transition(&mut self, index: usize) {
        if index >= self.count {
            return;
        }
        match self.states[index] {
            ComponentState::Transitioning(TransitionType::Init) => {
                self.states[index] = ComponentState::Initialized;
            }
            ComponentState::Transitioning(TransitionType::Run) => {
                self.states[index] = ComponentState::Running;
            }
            ComponentState::Transitioning(TransitionType::Shutdown) => {
                self.states[index] = ComponentState::ShutDown;
            }
            _ => {
                // Not in a transitioning state; no-op.
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Default
// ---------------------------------------------------------------------------

impl<const N: usize> Default for LifecycleManager<N> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{
        ComponentState, LifecycleComponent, LifecycleManager, ManagerState, TransitionResult,
        TransitionType,
    };

    // -----------------------------------------------------------------------
    // Mock component
    // -----------------------------------------------------------------------

    struct MockComponent {
        name: &'static str,
        init_result: TransitionResult,
        run_result: TransitionResult,
        shutdown_result: TransitionResult,
        init_called: bool,
        run_called: bool,
        shutdown_called: bool,
    }

    impl MockComponent {
        fn new(name: &'static str) -> Self {
            Self {
                name,
                init_result: TransitionResult::Done,
                run_result: TransitionResult::Done,
                shutdown_result: TransitionResult::Done,
                init_called: false,
                run_called: false,
                shutdown_called: false,
            }
        }

        fn with_init(mut self, result: TransitionResult) -> Self {
            self.init_result = result;
            self
        }

        fn with_run(mut self, result: TransitionResult) -> Self {
            self.run_result = result;
            self
        }

        fn with_shutdown(mut self, result: TransitionResult) -> Self {
            self.shutdown_result = result;
            self
        }
    }

    impl LifecycleComponent for MockComponent {
        fn init(&mut self) -> TransitionResult {
            self.init_called = true;
            self.init_result
        }

        fn run(&mut self) -> TransitionResult {
            self.run_called = true;
            self.run_result
        }

        fn shutdown(&mut self) -> TransitionResult {
            self.shutdown_called = true;
            self.shutdown_result
        }

        fn name(&self) -> &str {
            self.name
        }
    }

    // -----------------------------------------------------------------------
    // 1. New manager is Idle
    // -----------------------------------------------------------------------

    #[test]
    fn new_manager_is_idle() {
        let mgr: LifecycleManager<4> = LifecycleManager::new();
        assert_eq!(mgr.state(), ManagerState::Idle);
        assert_eq!(mgr.count(), 0);
    }

    // -----------------------------------------------------------------------
    // 2. Register component returns index
    // -----------------------------------------------------------------------

    #[test]
    fn register_returns_sequential_indices() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        assert_eq!(mgr.register(), Some(0));
        assert_eq!(mgr.register(), Some(1));
        assert_eq!(mgr.register(), Some(2));
        assert_eq!(mgr.count(), 3);
    }

    // -----------------------------------------------------------------------
    // 3. Register up to N components
    // -----------------------------------------------------------------------

    #[test]
    fn register_up_to_capacity() {
        let mut mgr: LifecycleManager<3> = LifecycleManager::new();
        assert_eq!(mgr.register(), Some(0));
        assert_eq!(mgr.register(), Some(1));
        assert_eq!(mgr.register(), Some(2));
        assert_eq!(mgr.count(), 3);
    }

    // -----------------------------------------------------------------------
    // 4. Register beyond N returns None
    // -----------------------------------------------------------------------

    #[test]
    fn register_beyond_capacity_returns_none() {
        let mut mgr: LifecycleManager<2> = LifecycleManager::new();
        assert_eq!(mgr.register(), Some(0));
        assert_eq!(mgr.register(), Some(1));
        assert_eq!(mgr.register(), None); // capacity exhausted
        assert_eq!(mgr.count(), 2);
    }

    // -----------------------------------------------------------------------
    // 5. init_component transitions to Initialized
    // -----------------------------------------------------------------------

    #[test]
    fn init_component_transitions_to_initialized() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        let idx = mgr.register().unwrap();
        let mut comp = MockComponent::new("A");

        let result = mgr.init_component(idx, &mut comp);

        assert_eq!(result, TransitionResult::Done);
        assert_eq!(mgr.component_state(idx), ComponentState::Initialized);
        assert!(comp.init_called);
    }

    // -----------------------------------------------------------------------
    // 6. run_component transitions to Running
    // -----------------------------------------------------------------------

    #[test]
    fn run_component_transitions_to_running() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        let idx = mgr.register().unwrap();
        let mut comp = MockComponent::new("A");

        mgr.init_component(idx, &mut comp);
        let result = mgr.run_component(idx, &mut comp);

        assert_eq!(result, TransitionResult::Done);
        assert_eq!(mgr.component_state(idx), ComponentState::Running);
        assert!(comp.run_called);
    }

    // -----------------------------------------------------------------------
    // 7. shutdown_component transitions to ShutDown
    // -----------------------------------------------------------------------

    #[test]
    fn shutdown_component_transitions_to_shutdown() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        let idx = mgr.register().unwrap();
        let mut comp = MockComponent::new("A");

        mgr.init_component(idx, &mut comp);
        mgr.run_component(idx, &mut comp);
        let result = mgr.shutdown_component(idx, &mut comp);

        assert_eq!(result, TransitionResult::Done);
        assert_eq!(mgr.component_state(idx), ComponentState::ShutDown);
        assert!(comp.shutdown_called);
    }

    // -----------------------------------------------------------------------
    // 8. init_all calls init on all registered components
    // -----------------------------------------------------------------------

    #[test]
    fn init_all_calls_init_on_all_registered() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        mgr.register().unwrap();
        mgr.register().unwrap();
        mgr.register().unwrap();

        let mut a = MockComponent::new("A");
        let mut b = MockComponent::new("B");
        let mut c = MockComponent::new("C");
        let mut comps: [&mut dyn LifecycleComponent; 3] = [&mut a, &mut b, &mut c];

        let pending = mgr.init_all(&mut comps);

        assert_eq!(pending, 0);
        assert_eq!(mgr.state(), ManagerState::InitInProgress);
        assert!(a.init_called);
        assert!(b.init_called);
        assert!(c.init_called);
        assert_eq!(mgr.component_state(0), ComponentState::Initialized);
        assert_eq!(mgr.component_state(1), ComponentState::Initialized);
        assert_eq!(mgr.component_state(2), ComponentState::Initialized);
    }

    // -----------------------------------------------------------------------
    // 9. run_all calls run on all initialized components
    // -----------------------------------------------------------------------

    #[test]
    fn run_all_calls_run_on_initialized_components() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        mgr.register().unwrap();
        mgr.register().unwrap();

        let mut a = MockComponent::new("A");
        let mut b = MockComponent::new("B");
        let mut comps: [&mut dyn LifecycleComponent; 2] = [&mut a, &mut b];

        mgr.init_all(&mut comps);
        let pending = mgr.run_all(&mut comps);

        assert_eq!(pending, 0);
        assert_eq!(mgr.state(), ManagerState::Running);
        assert!(a.run_called);
        assert!(b.run_called);
        assert_eq!(mgr.component_state(0), ComponentState::Running);
        assert_eq!(mgr.component_state(1), ComponentState::Running);
    }

    // -----------------------------------------------------------------------
    // 10. shutdown_all calls shutdown in reverse order
    // -----------------------------------------------------------------------

    #[test]
    fn shutdown_all_calls_shutdown_in_reverse_order() {
        use core::sync::atomic::{AtomicUsize, Ordering};

        struct OrderedMock {
            call_order: usize,
        }

        static GLOBAL_COUNTER: AtomicUsize = AtomicUsize::new(0);

        impl LifecycleComponent for OrderedMock {
            fn init(&mut self) -> TransitionResult {
                TransitionResult::Done
            }
            fn run(&mut self) -> TransitionResult {
                TransitionResult::Done
            }
            fn shutdown(&mut self) -> TransitionResult {
                self.call_order = GLOBAL_COUNTER.fetch_add(1, Ordering::Relaxed);
                TransitionResult::Done
            }
            fn name(&self) -> &str {
                "ordered"
            }
        }

        GLOBAL_COUNTER.store(0, Ordering::Relaxed);

        let mut mgr: LifecycleManager<3> = LifecycleManager::new();
        mgr.register().unwrap();
        mgr.register().unwrap();
        mgr.register().unwrap();

        let mut a = OrderedMock { call_order: 0 };
        let mut b = OrderedMock { call_order: 0 };
        let mut c = OrderedMock { call_order: 0 };
        let mut comps: [&mut dyn LifecycleComponent; 3] = [&mut a, &mut b, &mut c];

        mgr.init_all(&mut comps);
        mgr.run_all(&mut comps);
        let pending = mgr.shutdown_all(&mut comps);

        assert_eq!(pending, 0);
        assert_eq!(mgr.state(), ManagerState::ShutDown);
        // Reverse order: c first (call_order=0), b second (1), a last (2).
        assert_eq!(c.call_order, 0, "c should be shut down first");
        assert_eq!(b.call_order, 1, "b should be shut down second");
        assert_eq!(a.call_order, 2, "a should be shut down last");
    }

    // -----------------------------------------------------------------------
    // 11. Pending keeps component in Transitioning state
    // -----------------------------------------------------------------------

    #[test]
    fn pending_result_keeps_component_in_transitioning_state() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        let idx = mgr.register().unwrap();
        let mut comp = MockComponent::new("A").with_init(TransitionResult::Pending);

        let result = mgr.init_component(idx, &mut comp);

        assert_eq!(result, TransitionResult::Pending);
        assert_eq!(
            mgr.component_state(idx),
            ComponentState::Transitioning(TransitionType::Init)
        );
    }

    // -----------------------------------------------------------------------
    // 12. complete_transition advances from Transitioning
    // -----------------------------------------------------------------------

    #[test]
    fn complete_transition_advances_from_transitioning_init() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        let idx = mgr.register().unwrap();
        let mut comp = MockComponent::new("A").with_init(TransitionResult::Pending);

        mgr.init_component(idx, &mut comp);
        assert_eq!(
            mgr.component_state(idx),
            ComponentState::Transitioning(TransitionType::Init)
        );

        mgr.complete_transition(idx);
        assert_eq!(mgr.component_state(idx), ComponentState::Initialized);
    }

    #[test]
    fn complete_transition_advances_from_transitioning_run() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        let idx = mgr.register().unwrap();
        let mut comp = MockComponent::new("A").with_run(TransitionResult::Pending);

        mgr.init_component(idx, &mut comp);
        mgr.run_component(idx, &mut comp);
        assert_eq!(
            mgr.component_state(idx),
            ComponentState::Transitioning(TransitionType::Run)
        );

        mgr.complete_transition(idx);
        assert_eq!(mgr.component_state(idx), ComponentState::Running);
    }

    #[test]
    fn complete_transition_advances_from_transitioning_shutdown() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        let idx = mgr.register().unwrap();
        let mut comp = MockComponent::new("A").with_shutdown(TransitionResult::Pending);

        mgr.init_component(idx, &mut comp);
        mgr.run_component(idx, &mut comp);
        mgr.shutdown_component(idx, &mut comp);
        assert_eq!(
            mgr.component_state(idx),
            ComponentState::Transitioning(TransitionType::Shutdown)
        );

        mgr.complete_transition(idx);
        assert_eq!(mgr.component_state(idx), ComponentState::ShutDown);
    }

    // -----------------------------------------------------------------------
    // 13. Error result keeps component in current state
    // -----------------------------------------------------------------------

    #[test]
    fn error_result_leaves_state_unchanged_on_init() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        let idx = mgr.register().unwrap();
        let mut comp = MockComponent::new("A").with_init(TransitionResult::Error);

        let result = mgr.init_component(idx, &mut comp);

        assert_eq!(result, TransitionResult::Error);
        // State unchanged — still Uninitialized.
        assert_eq!(mgr.component_state(idx), ComponentState::Uninitialized);
    }

    #[test]
    fn error_result_leaves_state_unchanged_on_run() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        let idx = mgr.register().unwrap();
        let mut comp = MockComponent::new("A").with_run(TransitionResult::Error);

        mgr.init_component(idx, &mut comp);
        let result = mgr.run_component(idx, &mut comp);

        assert_eq!(result, TransitionResult::Error);
        // State unchanged — still Initialized.
        assert_eq!(mgr.component_state(idx), ComponentState::Initialized);
    }

    #[test]
    fn error_result_leaves_state_unchanged_on_shutdown() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        let idx = mgr.register().unwrap();
        let mut comp = MockComponent::new("A").with_shutdown(TransitionResult::Error);

        mgr.init_component(idx, &mut comp);
        mgr.run_component(idx, &mut comp);
        let result = mgr.shutdown_component(idx, &mut comp);

        assert_eq!(result, TransitionResult::Error);
        // State unchanged — still Running.
        assert_eq!(mgr.component_state(idx), ComponentState::Running);
    }

    // -----------------------------------------------------------------------
    // 14. Component state queries through full lifecycle
    // -----------------------------------------------------------------------

    #[test]
    fn component_state_queries_through_full_lifecycle() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        let idx = mgr.register().unwrap();
        let mut comp = MockComponent::new("A");

        assert_eq!(mgr.component_state(idx), ComponentState::Uninitialized);

        mgr.init_component(idx, &mut comp);
        assert_eq!(mgr.component_state(idx), ComponentState::Initialized);

        mgr.run_component(idx, &mut comp);
        assert_eq!(mgr.component_state(idx), ComponentState::Running);

        mgr.shutdown_component(idx, &mut comp);
        assert_eq!(mgr.component_state(idx), ComponentState::ShutDown);
    }

    #[test]
    fn component_state_out_of_bounds_returns_uninitialized() {
        let mgr: LifecycleManager<4> = LifecycleManager::new();
        // No components registered; index 0 is out of bounds.
        assert_eq!(mgr.component_state(0), ComponentState::Uninitialized);
        assert_eq!(mgr.component_state(99), ComponentState::Uninitialized);
    }

    // -----------------------------------------------------------------------
    // 15. Zero-capacity manager
    // -----------------------------------------------------------------------

    #[test]
    fn zero_capacity_manager() {
        let mut mgr: LifecycleManager<0> = LifecycleManager::new();
        assert_eq!(mgr.state(), ManagerState::Idle);
        assert_eq!(mgr.count(), 0);
        assert_eq!(mgr.register(), None);

        // Batch operations on empty slices must not panic.
        let mut comps: [&mut dyn LifecycleComponent; 0] = [];
        assert_eq!(mgr.init_all(&mut comps), 0);
        assert_eq!(mgr.run_all(&mut comps), 0);
        // shutdown_all on empty should advance to ShutDown immediately.
        assert_eq!(mgr.shutdown_all(&mut comps), 0);
        assert_eq!(mgr.state(), ManagerState::ShutDown);
    }

    // -----------------------------------------------------------------------
    // 16. Full lifecycle: init_all → run_all → shutdown_all
    // -----------------------------------------------------------------------

    #[test]
    fn full_lifecycle_init_run_shutdown() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        mgr.register().unwrap();
        mgr.register().unwrap();

        let mut a = MockComponent::new("A");
        let mut b = MockComponent::new("B");
        let mut comps: [&mut dyn LifecycleComponent; 2] = [&mut a, &mut b];

        // --- Init phase ---
        let pending = mgr.init_all(&mut comps);
        assert_eq!(pending, 0);
        assert_eq!(mgr.state(), ManagerState::InitInProgress);
        assert_eq!(mgr.component_state(0), ComponentState::Initialized);
        assert_eq!(mgr.component_state(1), ComponentState::Initialized);

        // --- Run phase ---
        let pending = mgr.run_all(&mut comps);
        assert_eq!(pending, 0);
        assert_eq!(mgr.state(), ManagerState::Running);
        assert_eq!(mgr.component_state(0), ComponentState::Running);
        assert_eq!(mgr.component_state(1), ComponentState::Running);

        // --- Shutdown phase ---
        let pending = mgr.shutdown_all(&mut comps);
        assert_eq!(pending, 0);
        assert_eq!(mgr.state(), ManagerState::ShutDown);
        assert_eq!(mgr.component_state(0), ComponentState::ShutDown);
        assert_eq!(mgr.component_state(1), ComponentState::ShutDown);

        // Verify all methods were called on both components.
        assert!(a.init_called && a.run_called && a.shutdown_called);
        assert!(b.init_called && b.run_called && b.shutdown_called);
    }

    // -----------------------------------------------------------------------
    // 17. init_all with pending components
    // -----------------------------------------------------------------------

    #[test]
    fn init_all_counts_pending_components() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        mgr.register().unwrap();
        mgr.register().unwrap();
        mgr.register().unwrap();

        let mut a = MockComponent::new("A").with_init(TransitionResult::Done);
        let mut b = MockComponent::new("B").with_init(TransitionResult::Pending);
        let mut c = MockComponent::new("C").with_init(TransitionResult::Pending);
        let mut comps: [&mut dyn LifecycleComponent; 3] = [&mut a, &mut b, &mut c];

        let pending = mgr.init_all(&mut comps);
        assert_eq!(pending, 2);
        assert_eq!(mgr.component_state(0), ComponentState::Initialized);
        assert_eq!(
            mgr.component_state(1),
            ComponentState::Transitioning(TransitionType::Init)
        );
        assert_eq!(
            mgr.component_state(2),
            ComponentState::Transitioning(TransitionType::Init)
        );
    }

    // -----------------------------------------------------------------------
    // 18. Default impl matches new()
    // -----------------------------------------------------------------------

    #[test]
    fn default_equals_new() {
        let a: LifecycleManager<4> = LifecycleManager::new();
        let b: LifecycleManager<4> = LifecycleManager::default();
        assert_eq!(a.state(), b.state());
        assert_eq!(a.count(), b.count());
    }

    // -----------------------------------------------------------------------
    // 19. init_component with out-of-bounds index returns Error
    // -----------------------------------------------------------------------

    #[test]
    fn init_component_out_of_bounds_returns_error() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        // No components registered.
        let mut comp = MockComponent::new("A");
        let result = mgr.init_component(0, &mut comp);
        assert_eq!(result, TransitionResult::Error);
        // init must NOT have been called on the component.
        assert!(!comp.init_called);
    }

    // -----------------------------------------------------------------------
    // 20. complete_transition on non-transitioning component is no-op
    // -----------------------------------------------------------------------

    #[test]
    fn complete_transition_on_non_transitioning_is_noop() {
        let mut mgr: LifecycleManager<4> = LifecycleManager::new();
        let idx = mgr.register().unwrap();
        // State: Uninitialized.
        mgr.complete_transition(idx); // must not panic or change state.
        assert_eq!(mgr.component_state(idx), ComponentState::Uninitialized);

        let mut comp = MockComponent::new("A");
        mgr.init_component(idx, &mut comp);
        // State: Initialized.
        mgr.complete_transition(idx); // must not panic or change state.
        assert_eq!(mgr.component_state(idx), ComponentState::Initialized);
    }
}
