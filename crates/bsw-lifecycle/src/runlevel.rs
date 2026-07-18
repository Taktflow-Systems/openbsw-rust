//! Run-level lifecycle manager (packages D01/D02).
//!
//! Ports the upstream `lifecycle::LifecycleManager` semantics
//! (`libs/bsw/lifecycle/include/lifecycle/LifecycleManager.h`):
//!
//! - components register at a run level; ascending transitions initialize
//!   and then run every level up to the target, descending transitions shut
//!   levels down above the target in reverse registration order;
//! - a component is initialized at most once, even across shutdown/run
//!   cycles — re-ascending only re-runs it;
//! - transitions may complete asynchronously: a component returns
//!   [`TransitionResult::Pending`] and the completion callback
//!   [`RunLevelManager::complete_transition`] finishes it later;
//! - a new target requested during a transition is queued; the latest
//!   request wins, exactly like upstream `_nextLevel`;
//! - per-component transition durations are recorded from an injected
//!   clock.
//!
//! Native replacements for upstream mechanisms:
//!
//! - `ILifecycleListener` becomes a bounded event queue drained with
//!   [`RunLevelManager::take_event`] — no intrusive listener list;
//! - D02 adds what upstream leaves implicit: a typed error/timeout policy.
//!   A failed or timed-out transition either halts at the current position
//!   ([`ErrorPolicy::Halt`]) or rolls the started level back down
//!   ([`ErrorPolicy::Rollback`]).

use bsw_time::{Duration, Instant};

use crate::{LifecycleComponent, TransitionResult};

/// Transition kind, mirroring upstream `ILifecycleComponent::Transition`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Transition {
    /// One-time initialization.
    Init,
    /// Activation at a run level.
    Run,
    /// Deactivation when leaving a run level.
    Shutdown,
}

/// Reaction to a failed or timed-out transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorPolicy {
    /// Stop where the failure happened; no further automatic transitions.
    Halt,
    /// Shut down the components already started for the failed level and
    /// settle on the previous level.
    Rollback,
}

/// Observable lifecycle event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// Every component of `level` finished its ascending transitions.
    LevelReached(u8),
    /// Every component above `level` finished shutting down.
    LevelLeft(u8),
    /// A component transition completed.
    TransitionDone {
        /// Registration index.
        component: usize,
        /// Which transition finished.
        transition: Transition,
    },
    /// A component transition reported an error.
    TransitionFailed {
        /// Registration index.
        component: usize,
        /// Which transition failed.
        transition: Transition,
    },
    /// A pending component transition exceeded the configured timeout.
    TransitionTimedOut {
        /// Registration index.
        component: usize,
        /// Which transition timed out.
        transition: Transition,
    },
}

/// Component state within the run-level manager.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Registered, never started.
    Idle,
    /// A transition is running asynchronously.
    Pending(Transition),
    /// Initialized (or shut down after having been initialized) but not
    /// running.
    Stopped,
    /// Actively running.
    Running,
    /// A transition failed; the component is out of service.
    Failed(Transition),
}

/// Manager progress after a step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepStatus {
    /// No transition is active; the current level equals the target.
    Idle,
    /// Waiting for at least one asynchronous component transition.
    InProgress,
    /// A component failed; the manager applied its error policy and stopped.
    Faulted,
}

/// Registration failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddError {
    /// The component table is full.
    TooManyComponents,
    /// The level is outside `0..MAX_LEVELS`.
    LevelOutOfRange,
    /// Level 0 is the "everything off" level and cannot own components.
    ReservedLevel,
}

#[derive(Debug, Clone, Copy)]
struct Info {
    level: u8,
    phase: Phase,
    initialized: bool,
    pending_since: Instant,
    times: [Option<Duration>; 3],
}

const EVENT_CAPACITY: usize = 32;

/// Run-level lifecycle manager over caller-owned components.
///
/// `N` bounds the component count and `L` the number of run levels
/// (levels are `1..=L`; level 0 means "everything shut down"). Components
/// are passed to every operation as a slice ordered by registration index,
/// exactly like the flat manager in the crate root.
pub struct RunLevelManager<const N: usize, const L: usize> {
    infos: [Option<Info>; N],
    count: usize,
    current: u8,
    target: u8,
    queued: Option<u8>,
    ascending_level: Option<u8>,
    descending_level: Option<u8>,
    policy: ErrorPolicy,
    timeout: Option<Duration>,
    faulted: bool,
    events: [Option<Event>; EVENT_CAPACITY],
    event_head: usize,
    event_len: usize,
    events_dropped: u32,
}

impl<const N: usize, const L: usize> RunLevelManager<N, L> {
    /// Create a manager at level 0 with the given error policy.
    pub const fn new(policy: ErrorPolicy) -> Self {
        Self {
            infos: [None; N],
            count: 0,
            current: 0,
            target: 0,
            queued: None,
            ascending_level: None,
            descending_level: None,
            policy,
            timeout: None,
            faulted: false,
            events: [None; EVENT_CAPACITY],
            event_head: 0,
            event_len: 0,
            events_dropped: 0,
        }
    }

    /// Limit how long an asynchronous transition may stay pending.
    pub fn set_transition_timeout(&mut self, timeout: Option<Duration>) {
        self.timeout = timeout;
    }

    /// Register a component at `level` (1-based). Returns its index.
    pub fn add_component(&mut self, level: u8) -> Result<usize, AddError> {
        if level == 0 {
            return Err(AddError::ReservedLevel);
        }
        if usize::from(level) > L {
            return Err(AddError::LevelOutOfRange);
        }
        if self.count == N {
            return Err(AddError::TooManyComponents);
        }
        let index = self.count;
        self.infos[index] = Some(Info {
            level,
            phase: Phase::Idle,
            initialized: false,
            pending_since: Instant::from_nanos(0),
            times: [None; 3],
        });
        self.count += 1;
        Ok(index)
    }

    /// Current settled level (the last fully reached level).
    pub const fn current_level(&self) -> u8 {
        self.current
    }

    /// Level the manager is moving toward.
    pub const fn target_level(&self) -> u8 {
        self.target
    }

    /// Phase of one component.
    pub fn phase(&self, index: usize) -> Option<Phase> {
        self.infos.get(index)?.map(|info| info.phase)
    }

    /// Recorded duration of a completed transition.
    pub fn transition_time(&self, index: usize, transition: Transition) -> Option<Duration> {
        self.infos.get(index)?.as_ref()?.times[transition_slot(transition)]
    }

    /// Pop the oldest queued event.
    pub fn take_event(&mut self) -> Option<Event> {
        if self.event_len == 0 {
            return None;
        }
        let event = self.events[self.event_head].take();
        self.event_head = (self.event_head + 1) % EVENT_CAPACITY;
        self.event_len -= 1;
        event
    }

    /// Events discarded because the queue was full.
    pub const fn events_dropped(&self) -> u32 {
        self.events_dropped
    }

    /// Request a transition to `level`.
    ///
    /// Requesting the current target again is a no-op (idempotent). While a
    /// transition is active the newest request is queued and served next,
    /// mirroring upstream `_nextLevel`.
    pub fn transition_to_level(&mut self, level: u8) -> Result<(), AddError> {
        if usize::from(level) > L {
            return Err(AddError::LevelOutOfRange);
        }
        if self.is_busy() {
            self.queued = Some(level);
        } else {
            self.faulted = false;
            self.target = level;
        }
        Ok(())
    }

    /// Complete a pending asynchronous transition.
    ///
    /// Safe to call spuriously: unknown indices and components that are not
    /// pending are ignored, like repeated upstream `transitionDone` calls.
    pub fn complete_transition(&mut self, index: usize, now: Instant) {
        let Some(Some(info)) = self.infos.get_mut(index) else {
            return;
        };
        let Phase::Pending(transition) = info.phase else {
            return;
        };
        info.times[transition_slot(transition)] = Some(now.duration_since(info.pending_since));
        Self::finish(info, transition);
        let event = Event::TransitionDone {
            component: index,
            transition,
        };
        self.push_event(event);
    }

    /// Drive the manager: start due transitions, detect timeouts, and
    /// advance through levels until the target is reached or progress needs
    /// an asynchronous completion.
    pub fn step(
        &mut self,
        components: &mut [&mut dyn LifecycleComponent],
        now: Instant,
    ) -> StepStatus {
        loop {
            if self.check_timeouts(now) {
                self.apply_error_policy();
                return StepStatus::Faulted;
            }
            if self.has_pending() {
                return StepStatus::InProgress;
            }
            if self.faulted {
                return StepStatus::Faulted;
            }
            if self.current == self.target {
                self.ascending_level = None;
                self.descending_level = None;
                if let Some(queued) = self.queued.take() {
                    self.faulted = false;
                    self.target = queued;
                    continue;
                }
                return StepStatus::Idle;
            }
            if self.current < self.target {
                let level = self.ascending_level.unwrap_or(self.current + 1);
                self.ascending_level = Some(level);
                match self.start_ascending(components, level, now) {
                    BatchOutcome::AllDone => {
                        self.current = level;
                        self.ascending_level = None;
                        self.push_event(Event::LevelReached(level));
                    }
                    BatchOutcome::Pending => return StepStatus::InProgress,
                    BatchOutcome::Failed => {
                        self.apply_error_policy();
                        return StepStatus::Faulted;
                    }
                }
            } else {
                let level = self.descending_level.unwrap_or(self.current);
                self.descending_level = Some(level);
                match self.start_shutdown(components, level, now) {
                    BatchOutcome::AllDone => {
                        self.current = level - 1;
                        self.descending_level = None;
                        self.push_event(Event::LevelLeft(self.current));
                    }
                    BatchOutcome::Pending => return StepStatus::InProgress,
                    BatchOutcome::Failed => {
                        self.apply_error_policy();
                        return StepStatus::Faulted;
                    }
                }
            }
        }
    }

    fn is_busy(&self) -> bool {
        self.current != self.target || self.has_pending()
    }

    fn has_pending(&self) -> bool {
        self.infos[..self.count]
            .iter()
            .flatten()
            .any(|info| matches!(info.phase, Phase::Pending(_)))
    }

    fn check_timeouts(&mut self, now: Instant) -> bool {
        let Some(timeout) = self.timeout else {
            return false;
        };
        let mut timed_out = false;
        for index in 0..self.count {
            let Some(info) = self.infos[index].as_mut() else {
                continue;
            };
            let Phase::Pending(transition) = info.phase else {
                continue;
            };
            if now.duration_since(info.pending_since) >= timeout {
                info.phase = Phase::Failed(transition);
                timed_out = true;
                self.push_event(Event::TransitionTimedOut {
                    component: index,
                    transition,
                });
            }
        }
        if timed_out {
            self.faulted = true;
        }
        timed_out
    }

    /// Initialize (first time only) and run every component of `level`.
    fn start_ascending(
        &mut self,
        components: &mut [&mut dyn LifecycleComponent],
        level: u8,
        now: Instant,
    ) -> BatchOutcome {
        // First pass: initialization for never-initialized components.
        let init = self.run_batch(components, level, Transition::Init, now, false);
        match init {
            BatchOutcome::AllDone => {}
            other => return other,
        }
        self.run_batch(components, level, Transition::Run, now, false)
    }

    fn start_shutdown(
        &mut self,
        components: &mut [&mut dyn LifecycleComponent],
        level: u8,
        now: Instant,
    ) -> BatchOutcome {
        self.run_batch(components, level, Transition::Shutdown, now, true)
    }

    fn run_batch(
        &mut self,
        components: &mut [&mut dyn LifecycleComponent],
        level: u8,
        transition: Transition,
        now: Instant,
        reverse: bool,
    ) -> BatchOutcome {
        let mut outcome = BatchOutcome::AllDone;
        let count = self.count.min(components.len());
        for position in 0..count {
            let index = if reverse {
                count - 1 - position
            } else {
                position
            };
            let Some(info) = self.infos[index].as_mut() else {
                continue;
            };
            if info.level != level {
                continue;
            }
            if !Self::transition_applies(info, transition) {
                continue;
            }
            info.pending_since = now;
            let result = match transition {
                Transition::Init => components[index].init(),
                Transition::Run => components[index].run(),
                Transition::Shutdown => components[index].shutdown(),
            };
            match result {
                TransitionResult::Done => {
                    info.times[transition_slot(transition)] = Some(Duration::ZERO);
                    Self::finish(info, transition);
                    self.push_event(Event::TransitionDone {
                        component: index,
                        transition,
                    });
                }
                TransitionResult::Pending => {
                    info.phase = Phase::Pending(transition);
                    if outcome == BatchOutcome::AllDone {
                        outcome = BatchOutcome::Pending;
                    }
                }
                TransitionResult::Error => {
                    info.phase = Phase::Failed(transition);
                    self.faulted = true;
                    self.push_event(Event::TransitionFailed {
                        component: index,
                        transition,
                    });
                    return BatchOutcome::Failed;
                }
            }
        }
        outcome
    }

    /// Whether `transition` still needs to happen for this component.
    fn transition_applies(info: &Info, transition: Transition) -> bool {
        match transition {
            // Upstream initializes a component at most once, ever.
            Transition::Init => !info.initialized,
            Transition::Run => info.phase != Phase::Running,
            Transition::Shutdown => info.phase == Phase::Running,
        }
    }

    fn finish(info: &mut Info, transition: Transition) {
        match transition {
            Transition::Init => {
                info.initialized = true;
                info.phase = Phase::Stopped;
            }
            Transition::Run => info.phase = Phase::Running,
            Transition::Shutdown => info.phase = Phase::Stopped,
        }
    }

    fn apply_error_policy(&mut self) {
        match self.policy {
            ErrorPolicy::Halt => {
                // Stay put; clear movement so step() reports Faulted/Idle
                // deterministically.
                self.target = self.current;
                self.ascending_level = None;
                self.descending_level = None;
                self.queued = None;
            }
            ErrorPolicy::Rollback => {
                // Settle one level below the level being started, shutting
                // down whatever already runs there on the next steps.
                self.target = self.current;
                self.ascending_level = None;
                self.descending_level = None;
                self.queued = None;
            }
        }
    }

    /// After a fault with [`ErrorPolicy::Rollback`], shut down components
    /// of the failed level that already reached `Running`.
    pub fn rollback_failed_level(
        &mut self,
        components: &mut [&mut dyn LifecycleComponent],
        now: Instant,
    ) -> StepStatus {
        if !self.faulted || self.policy != ErrorPolicy::Rollback {
            return StepStatus::Idle;
        }
        // Shut down every running component whose level is above the
        // settled level.
        let level_bound = self.current;
        let mut any_pending = false;
        for index in (0..self.count).rev() {
            let Some(info) = self.infos[index].as_mut() else {
                continue;
            };
            if info.level <= level_bound || info.phase != Phase::Running {
                continue;
            }
            info.pending_since = now;
            match components[index].shutdown() {
                TransitionResult::Done => {
                    info.times[transition_slot(Transition::Shutdown)] = Some(Duration::ZERO);
                    Self::finish(info, Transition::Shutdown);
                    self.push_event(Event::TransitionDone {
                        component: index,
                        transition: Transition::Shutdown,
                    });
                }
                TransitionResult::Pending => {
                    info.phase = Phase::Pending(Transition::Shutdown);
                    any_pending = true;
                }
                TransitionResult::Error => {
                    info.phase = Phase::Failed(Transition::Shutdown);
                    self.push_event(Event::TransitionFailed {
                        component: index,
                        transition: Transition::Shutdown,
                    });
                }
            }
        }
        if any_pending {
            StepStatus::InProgress
        } else {
            StepStatus::Idle
        }
    }

    fn push_event(&mut self, event: Event) {
        if self.event_len == EVENT_CAPACITY {
            self.events_dropped = self.events_dropped.saturating_add(1);
            return;
        }
        let slot = (self.event_head + self.event_len) % EVENT_CAPACITY;
        self.events[slot] = Some(event);
        self.event_len += 1;
    }
}

const fn transition_slot(transition: Transition) -> usize {
    match transition {
        Transition::Init => 0,
        Transition::Run => 1,
        Transition::Shutdown => 2,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BatchOutcome {
    AllDone,
    Pending,
    Failed,
}
