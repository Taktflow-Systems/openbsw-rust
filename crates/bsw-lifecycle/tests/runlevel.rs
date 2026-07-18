//! Run-level manager scenarios (packages D01/D02), pinned to upstream
//! `lifecycle::LifecycleManager` semantics.

use bsw_lifecycle::runlevel::{
    AddError, ErrorPolicy, Event, Phase, RunLevelManager, StepStatus, Transition,
};
use bsw_lifecycle::{LifecycleComponent, TransitionResult};
use bsw_time::{Duration, Instant};
use std::cell::RefCell;
use std::rc::Rc;

/// Scripted component recording every call into a shared journal.
struct Probe {
    name: &'static str,
    journal: Rc<RefCell<Vec<String>>>,
    init_result: TransitionResult,
    run_result: TransitionResult,
    shutdown_result: TransitionResult,
}

impl Probe {
    fn new(name: &'static str, journal: &Rc<RefCell<Vec<String>>>) -> Self {
        Self {
            name,
            journal: Rc::clone(journal),
            init_result: TransitionResult::Done,
            run_result: TransitionResult::Done,
            shutdown_result: TransitionResult::Done,
        }
    }
}

impl LifecycleComponent for Probe {
    fn init(&mut self) -> TransitionResult {
        self.journal
            .borrow_mut()
            .push(format!("init {}", self.name));
        self.init_result
    }

    fn run(&mut self) -> TransitionResult {
        self.journal.borrow_mut().push(format!("run {}", self.name));
        self.run_result
    }

    fn shutdown(&mut self) -> TransitionResult {
        self.journal
            .borrow_mut()
            .push(format!("shutdown {}", self.name));
        self.shutdown_result
    }

    fn name(&self) -> &str {
        self.name
    }
}

fn at(nanos: u64) -> Instant {
    Instant::from_nanos(nanos)
}

fn drain_events<const N: usize, const L: usize>(manager: &mut RunLevelManager<N, L>) -> Vec<Event> {
    let mut events = Vec::new();
    while let Some(event) = manager.take_event() {
        events.push(event);
    }
    events
}

#[test]
fn ascending_initializes_then_runs_levels_in_order() {
    let journal = Rc::new(RefCell::new(Vec::new()));
    let mut a = Probe::new("a", &journal);
    let mut b = Probe::new("b", &journal);
    let mut c = Probe::new("c", &journal);
    let mut components: [&mut dyn LifecycleComponent; 3] = [&mut a, &mut b, &mut c];
    let mut manager: RunLevelManager<3, 3> = RunLevelManager::new(ErrorPolicy::Halt);
    manager.add_component(1).unwrap();
    manager.add_component(1).unwrap();
    manager.add_component(2).unwrap();
    manager.transition_to_level(2).unwrap();
    assert_eq!(manager.step(&mut components, at(0)), StepStatus::Idle);
    assert_eq!(
        *journal.borrow(),
        ["init a", "init b", "run a", "run b", "init c", "run c"]
    );
    assert_eq!(manager.current_level(), 2);
    let events = drain_events(&mut manager);
    assert!(events.contains(&Event::LevelReached(1)));
    assert!(events.contains(&Event::LevelReached(2)));
}

#[test]
fn descending_shuts_down_levels_in_reverse_registration_order() {
    let journal = Rc::new(RefCell::new(Vec::new()));
    let mut a = Probe::new("a", &journal);
    let mut b = Probe::new("b", &journal);
    let mut c = Probe::new("c", &journal);
    let mut components: [&mut dyn LifecycleComponent; 3] = [&mut a, &mut b, &mut c];
    let mut manager: RunLevelManager<3, 2> = RunLevelManager::new(ErrorPolicy::Halt);
    manager.add_component(1).unwrap();
    manager.add_component(1).unwrap();
    manager.add_component(2).unwrap();
    manager.transition_to_level(2).unwrap();
    manager.step(&mut components, at(0));
    journal.borrow_mut().clear();
    manager.transition_to_level(0).unwrap();
    assert_eq!(manager.step(&mut components, at(1)), StepStatus::Idle);
    assert_eq!(
        *journal.borrow(),
        ["shutdown c", "shutdown b", "shutdown a"]
    );
    assert_eq!(manager.current_level(), 0);
}

#[test]
fn components_initialize_at_most_once_across_cycles() {
    let journal = Rc::new(RefCell::new(Vec::new()));
    let mut a = Probe::new("a", &journal);
    let mut components: [&mut dyn LifecycleComponent; 1] = [&mut a];
    let mut manager: RunLevelManager<1, 1> = RunLevelManager::new(ErrorPolicy::Halt);
    manager.add_component(1).unwrap();
    manager.transition_to_level(1).unwrap();
    manager.step(&mut components, at(0));
    manager.transition_to_level(0).unwrap();
    manager.step(&mut components, at(1));
    manager.transition_to_level(1).unwrap();
    manager.step(&mut components, at(2));
    // Upstream: init happens exactly once; re-ascending only reruns.
    assert_eq!(
        *journal.borrow(),
        ["init a", "run a", "shutdown a", "run a"]
    );
}

#[test]
fn pending_transitions_finish_through_completion_callback() {
    let journal = Rc::new(RefCell::new(Vec::new()));
    let mut a = Probe::new("a", &journal);
    a.init_result = TransitionResult::Pending;
    let mut b = Probe::new("b", &journal);
    let mut components: [&mut dyn LifecycleComponent; 2] = [&mut a, &mut b];
    let mut manager: RunLevelManager<2, 1> = RunLevelManager::new(ErrorPolicy::Halt);
    manager.add_component(1).unwrap();
    manager.add_component(1).unwrap();
    manager.transition_to_level(1).unwrap();
    assert_eq!(
        manager.step(&mut components, at(100)),
        StepStatus::InProgress
    );
    // Both inits started; only b finished synchronously.
    assert_eq!(*journal.borrow(), ["init a", "init b"]);
    assert_eq!(manager.phase(0), Some(Phase::Pending(Transition::Init)));
    // Spurious completions are ignored.
    manager.complete_transition(1, at(150));
    manager.complete_transition(9, at(150));
    manager.complete_transition(0, at(400));
    assert_eq!(manager.step(&mut components, at(500)), StepStatus::Idle);
    assert_eq!(*journal.borrow(), ["init a", "init b", "run a", "run b"]);
    assert_eq!(
        manager.transition_time(0, Transition::Init),
        Some(Duration::from_nanos(300))
    );
    assert_eq!(manager.current_level(), 1);
}

#[test]
fn newest_queued_target_wins_during_pending_transition() {
    let journal = Rc::new(RefCell::new(Vec::new()));
    let mut a = Probe::new("a", &journal);
    a.init_result = TransitionResult::Pending;
    let mut components: [&mut dyn LifecycleComponent; 1] = [&mut a];
    let mut manager: RunLevelManager<1, 2> = RunLevelManager::new(ErrorPolicy::Halt);
    manager.add_component(1).unwrap();
    manager.transition_to_level(1).unwrap();
    assert_eq!(manager.step(&mut components, at(0)), StepStatus::InProgress);
    // Two queued requests: the newest (level 0) wins.
    manager.transition_to_level(2).unwrap();
    manager.transition_to_level(0).unwrap();
    manager.complete_transition(0, at(10));
    assert_eq!(manager.step(&mut components, at(20)), StepStatus::Idle);
    assert_eq!(manager.current_level(), 0);
    assert_eq!(*journal.borrow(), ["init a", "run a", "shutdown a"]);
}

#[test]
fn requesting_the_current_level_is_idempotent() {
    let journal = Rc::new(RefCell::new(Vec::new()));
    let mut a = Probe::new("a", &journal);
    let mut components: [&mut dyn LifecycleComponent; 1] = [&mut a];
    let mut manager: RunLevelManager<1, 1> = RunLevelManager::new(ErrorPolicy::Halt);
    manager.add_component(1).unwrap();
    manager.transition_to_level(1).unwrap();
    manager.step(&mut components, at(0));
    journal.borrow_mut().clear();
    manager.transition_to_level(1).unwrap();
    assert_eq!(manager.step(&mut components, at(1)), StepStatus::Idle);
    assert!(journal.borrow().is_empty());
}

#[test]
fn init_failure_halts_and_reports() {
    let journal = Rc::new(RefCell::new(Vec::new()));
    let mut a = Probe::new("a", &journal);
    a.init_result = TransitionResult::Error;
    let mut b = Probe::new("b", &journal);
    let mut components: [&mut dyn LifecycleComponent; 2] = [&mut a, &mut b];
    let mut manager: RunLevelManager<2, 1> = RunLevelManager::new(ErrorPolicy::Halt);
    manager.add_component(1).unwrap();
    manager.add_component(1).unwrap();
    manager.transition_to_level(1).unwrap();
    assert_eq!(manager.step(&mut components, at(0)), StepStatus::Faulted);
    assert_eq!(manager.phase(0), Some(Phase::Failed(Transition::Init)));
    assert_eq!(manager.current_level(), 0);
    // The failure stops the batch: b was never touched.
    assert_eq!(*journal.borrow(), ["init a"]);
    let events = drain_events(&mut manager);
    assert!(events.contains(&Event::TransitionFailed {
        component: 0,
        transition: Transition::Init,
    }));
    // The fault is sticky and no transition is retried implicitly.
    assert_eq!(manager.step(&mut components, at(1)), StepStatus::Faulted);
    assert_eq!(*journal.borrow(), ["init a"]);
    // An explicit new request clears the fault and retries.
    manager.transition_to_level(1).unwrap();
    assert_eq!(manager.step(&mut components, at(2)), StepStatus::Faulted);
    assert_eq!(*journal.borrow(), ["init a", "init a"]);
}

#[test]
fn rollback_policy_shuts_down_partially_started_level() {
    let journal = Rc::new(RefCell::new(Vec::new()));
    let mut a = Probe::new("a", &journal);
    let mut b = Probe::new("b", &journal);
    b.run_result = TransitionResult::Error;
    let mut components: [&mut dyn LifecycleComponent; 2] = [&mut a, &mut b];
    let mut manager: RunLevelManager<2, 1> = RunLevelManager::new(ErrorPolicy::Rollback);
    manager.add_component(1).unwrap();
    manager.add_component(1).unwrap();
    manager.transition_to_level(1).unwrap();
    assert_eq!(manager.step(&mut components, at(0)), StepStatus::Faulted);
    // a reached Running before b failed; rollback shuts a down again.
    assert_eq!(
        manager.rollback_failed_level(&mut components, at(1)),
        StepStatus::Idle
    );
    assert_eq!(
        *journal.borrow(),
        ["init a", "init b", "run a", "run b", "shutdown a"]
    );
    assert_eq!(manager.phase(0), Some(Phase::Stopped));
    assert_eq!(manager.phase(1), Some(Phase::Failed(Transition::Run)));
    assert_eq!(manager.current_level(), 0);
}

#[test]
fn pending_transition_times_out_deterministically() {
    let journal = Rc::new(RefCell::new(Vec::new()));
    let mut a = Probe::new("a", &journal);
    a.init_result = TransitionResult::Pending;
    let mut components: [&mut dyn LifecycleComponent; 1] = [&mut a];
    let mut manager: RunLevelManager<1, 1> = RunLevelManager::new(ErrorPolicy::Halt);
    manager.add_component(1).unwrap();
    manager.set_transition_timeout(Some(Duration::from_millis(5).unwrap()));
    manager.transition_to_level(1).unwrap();
    assert_eq!(manager.step(&mut components, at(0)), StepStatus::InProgress);
    // One nanosecond before the deadline: still in progress.
    let before = at(5_000_000 - 1);
    assert_eq!(
        manager.step(&mut components, before),
        StepStatus::InProgress
    );
    let deadline = at(5_000_000);
    assert_eq!(manager.step(&mut components, deadline), StepStatus::Faulted);
    assert_eq!(manager.phase(0), Some(Phase::Failed(Transition::Init)));
    let events = drain_events(&mut manager);
    assert!(events.contains(&Event::TransitionTimedOut {
        component: 0,
        transition: Transition::Init,
    }));
    // A completion arriving after the timeout is ignored.
    manager.complete_transition(0, at(6_000_000));
    assert_eq!(manager.phase(0), Some(Phase::Failed(Transition::Init)));
}

#[test]
fn registration_is_validated() {
    let mut manager: RunLevelManager<1, 2> = RunLevelManager::new(ErrorPolicy::Halt);
    assert_eq!(manager.add_component(0), Err(AddError::ReservedLevel));
    assert_eq!(manager.add_component(3), Err(AddError::LevelOutOfRange));
    manager.add_component(2).unwrap();
    assert_eq!(manager.add_component(1), Err(AddError::TooManyComponents));
    assert_eq!(
        manager.transition_to_level(7),
        Err(AddError::LevelOutOfRange)
    );
}

#[test]
fn synchronous_transition_times_are_recorded() {
    let journal = Rc::new(RefCell::new(Vec::new()));
    let mut a = Probe::new("a", &journal);
    let mut components: [&mut dyn LifecycleComponent; 1] = [&mut a];
    let mut manager: RunLevelManager<1, 1> = RunLevelManager::new(ErrorPolicy::Halt);
    manager.add_component(1).unwrap();
    manager.transition_to_level(1).unwrap();
    manager.step(&mut components, at(42));
    assert_eq!(
        manager.transition_time(0, Transition::Init),
        Some(Duration::ZERO)
    );
    assert_eq!(
        manager.transition_time(0, Transition::Run),
        Some(Duration::ZERO)
    );
    assert_eq!(manager.transition_time(0, Transition::Shutdown), None);
}
