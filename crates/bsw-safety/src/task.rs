//! Composed safety task with deterministic fault injection.

use bsw_lifecycle::{LifecycleComponent, TransitionResult};
use bsw_time::Instant;

use crate::mechanisms::{RomCheckStatus, RomCrcChecker};
use crate::monitor::WatchdogMonitor;
use crate::supervisor::{SafeSupervisor, SafetyEvent, SafetyEventCode, SafetyPolicy, SafetySink};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultInjection {
    None,
    SkipWatchdogKick,
    RomMismatch,
    MonitorFailure(u16),
}

pub struct SafetyTask<'a, P: SafetyPolicy, S: SafetySink> {
    supervisor: SafeSupervisor<P, S>,
    watchdog: WatchdogMonitor<SafetyEvent, u16>,
    rom: RomCrcChecker<'a>,
    injection: FaultInjection,
    initialized: bool,
    shutdown: bool,
}

impl<'a, P: SafetyPolicy, S: SafetySink> SafetyTask<'a, P, S> {
    pub fn new(
        supervisor: SafeSupervisor<P, S>,
        rom: RomCrcChecker<'a>,
        watchdog_cycles: u32,
    ) -> Option<Self> {
        Some(Self {
            supervisor,
            watchdog: WatchdogMonitor::new(
                SafetyEvent {
                    code: SafetyEventCode::Watchdog,
                    detail: 1,
                    timestamp_ns: 0,
                },
                watchdog_cycles,
            )?,
            rom,
            injection: FaultInjection::None,
            initialized: false,
            shutdown: false,
        })
    }
    pub fn inject(&mut self, fault: FaultInjection) {
        self.injection = fault;
    }
    pub fn step(&mut self, now: Instant, healthy_checkpoint: bool) -> RomCheckStatus {
        if !self.initialized || self.shutdown {
            return self.rom.status();
        }
        if healthy_checkpoint && self.injection != FaultInjection::SkipWatchdogKick {
            self.watchdog.kick(0);
        }
        let mut event_handler = TimestampedHandler {
            supervisor: &mut self.supervisor,
            now,
        };
        let _ = self.watchdog.service(&mut event_handler);
        if let FaultInjection::MonitorFailure(detail) = self.injection {
            event_handler.supervisor.route(SafetyEvent {
                code: SafetyEventCode::Monitor,
                detail,
                timestamp_ns: now.as_nanos(),
            });
        }
        let status = match self.injection {
            FaultInjection::RomMismatch => RomCheckStatus::Failed,
            _ => self.rom.step(),
        };
        if status == RomCheckStatus::Failed {
            event_handler.supervisor.route(SafetyEvent {
                code: SafetyEventCode::RomCrc,
                detail: 1,
                timestamp_ns: now.as_nanos(),
            });
        }
        status
    }
    pub const fn supervisor(&self) -> &SafeSupervisor<P, S> {
        &self.supervisor
    }

    pub fn route(&mut self, event: SafetyEvent) {
        self.supervisor.route(event);
    }
}

struct TimestampedHandler<'a, P: SafetyPolicy, S: SafetySink> {
    supervisor: &'a mut SafeSupervisor<P, S>,
    now: Instant,
}
impl<P: SafetyPolicy, S: SafetySink> crate::monitor::EventHandler<SafetyEvent>
    for TimestampedHandler<'_, P, S>
{
    fn handle(&mut self, mut event: SafetyEvent) {
        event.timestamp_ns = self.now.as_nanos();
        self.supervisor.route(event);
    }
}

impl<P: SafetyPolicy, S: SafetySink> LifecycleComponent for SafetyTask<'_, P, S> {
    fn init(&mut self) -> TransitionResult {
        self.initialized = true;
        self.shutdown = false;
        TransitionResult::Done
    }
    fn run(&mut self) -> TransitionResult {
        if self.initialized && !self.shutdown {
            TransitionResult::Done
        } else {
            TransitionResult::Error
        }
    }
    fn shutdown(&mut self) -> TransitionResult {
        self.shutdown = true;
        TransitionResult::Done
    }
    fn name(&self) -> &str {
        "SafetyTask"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::supervisor::{FailureAction, SafetySeverity};
    use bsw_util::crc::CRC32_ETHERNET;
    struct Policy;
    impl SafetyPolicy for Policy {
        fn severity(&self, _: &SafetyEvent) -> SafetySeverity {
            SafetySeverity::Critical
        }
        fn action(&self, _: &SafetyEvent, _: SafetySeverity) -> FailureAction {
            FailureAction::ControlledReset
        }
    }
    #[derive(Default)]
    struct Sink {
        records: u32,
        resets: u32,
    }
    impl SafetySink for Sink {
        fn record(&mut self, _: SafetyEvent, _: SafetySeverity) {
            self.records += 1;
        }
        fn enter_limp_home(&mut self, _: SafetyEvent) {}
        fn request_reset(&mut self, _: SafetyEvent) {
            self.resets += 1;
        }
    }
    #[test]
    fn lifecycle_under_load_and_faults_is_deterministic() {
        let image = b"rom";
        let rom = RomCrcChecker::new(image, CRC32_ETHERNET.checksum(image), 1).unwrap();
        let mut task =
            SafetyTask::new(SafeSupervisor::new(Policy, Sink::default()), rom, 2).unwrap();
        assert_eq!(task.init(), TransitionResult::Done);
        assert_eq!(
            task.step(Instant::from_nanos(1), true),
            RomCheckStatus::InProgress
        );
        task.inject(FaultInjection::SkipWatchdogKick);
        let _ = task.step(Instant::from_nanos(2), false);
        let _ = task.step(Instant::from_nanos(3), false);
        assert_eq!(task.supervisor().reset_requests(), 2);
        task.inject(FaultInjection::RomMismatch);
        assert_eq!(
            task.step(Instant::from_nanos(4), true),
            RomCheckStatus::Failed
        );
        assert_eq!(task.supervisor().reset_requests(), 3);
        assert_eq!(task.shutdown(), TransitionResult::Done);
        assert_eq!(task.run(), TransitionResult::Error);
    }
}
