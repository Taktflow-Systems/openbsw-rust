//! Injected safety-event severity and failure-routing policy.

use crate::monitor::EventHandler;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SafetySeverity {
    Information = 0,
    Degraded = 1,
    Critical = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FailureAction {
    LogOnly = 0,
    LimpHome = 1,
    ControlledReset = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum SafetyEventCode {
    Monitor = 1,
    Watchdog = 2,
    RetainedRecord = 3,
    RomCrc = 4,
    MpuConfiguration = 5,
    Ecc = 6,
    Lifecycle = 7,
    Reset = 8,
    HardFault = 9,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SafetyEvent {
    pub code: SafetyEventCode,
    pub detail: u16,
    pub timestamp_ns: u64,
}

pub trait SafetyPolicy {
    fn severity(&self, event: &SafetyEvent) -> SafetySeverity;
    fn action(&self, event: &SafetyEvent, severity: SafetySeverity) -> FailureAction;
}

pub trait SafetySink {
    fn record(&mut self, event: SafetyEvent, severity: SafetySeverity);
    fn enter_limp_home(&mut self, event: SafetyEvent);
    fn request_reset(&mut self, event: SafetyEvent);
}

pub struct SafeSupervisor<P, S> {
    policy: P,
    sink: S,
    events: u32,
    limp_home_requests: u32,
    reset_requests: u32,
}

impl<P: SafetyPolicy, S: SafetySink> SafeSupervisor<P, S> {
    pub const fn new(policy: P, sink: S) -> Self {
        Self {
            policy,
            sink,
            events: 0,
            limp_home_requests: 0,
            reset_requests: 0,
        }
    }
    pub fn route(&mut self, event: SafetyEvent) -> FailureAction {
        let severity = self.policy.severity(&event);
        let action = self.policy.action(&event, severity);
        self.sink.record(event, severity);
        self.events = self.events.saturating_add(1);
        match action {
            FailureAction::LogOnly => {}
            FailureAction::LimpHome => {
                self.limp_home_requests = self.limp_home_requests.saturating_add(1);
                self.sink.enter_limp_home(event);
            }
            FailureAction::ControlledReset => {
                self.reset_requests = self.reset_requests.saturating_add(1);
                self.sink.request_reset(event);
            }
        }
        action
    }
    pub const fn event_count(&self) -> u32 {
        self.events
    }
    pub const fn limp_home_requests(&self) -> u32 {
        self.limp_home_requests
    }
    pub const fn reset_requests(&self) -> u32 {
        self.reset_requests
    }
    pub const fn sink(&self) -> &S {
        &self.sink
    }
    pub fn sink_mut(&mut self) -> &mut S {
        &mut self.sink
    }
}

impl<P: SafetyPolicy, S: SafetySink> EventHandler<SafetyEvent> for SafeSupervisor<P, S> {
    fn handle(&mut self, event: SafetyEvent) {
        self.route(event);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    struct Policy;
    impl SafetyPolicy for Policy {
        fn severity(&self, event: &SafetyEvent) -> SafetySeverity {
            if event.detail >= 10 {
                SafetySeverity::Critical
            } else {
                SafetySeverity::Degraded
            }
        }
        fn action(&self, _event: &SafetyEvent, severity: SafetySeverity) -> FailureAction {
            if severity == SafetySeverity::Critical {
                FailureAction::ControlledReset
            } else {
                FailureAction::LimpHome
            }
        }
    }
    #[derive(Default)]
    struct Sink {
        records: u32,
        limp: u32,
        reset: u32,
    }
    impl SafetySink for Sink {
        fn record(&mut self, _: SafetyEvent, _: SafetySeverity) {
            self.records += 1;
        }
        fn enter_limp_home(&mut self, _: SafetyEvent) {
            self.limp += 1;
        }
        fn request_reset(&mut self, _: SafetyEvent) {
            self.reset += 1;
        }
    }
    #[test]
    fn injected_policy_routes_every_severity() {
        let mut s = SafeSupervisor::new(Policy, Sink::default());
        let event = |detail| SafetyEvent {
            code: SafetyEventCode::Monitor,
            detail,
            timestamp_ns: detail.into(),
        };
        assert_eq!(s.route(event(1)), FailureAction::LimpHome);
        assert_eq!(s.route(event(10)), FailureAction::ControlledReset);
        assert_eq!(
            (s.event_count(), s.limp_home_requests(), s.reset_requests()),
            (2, 1, 1)
        );
        assert_eq!((s.sink().records, s.sink().limp, s.sink().reset), (2, 1, 1));
    }
}
