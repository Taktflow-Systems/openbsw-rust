//! Heap-free reference-application state shared by POSIX, F413, and G474.

#![no_std]

pub mod diagnostics;
pub mod io;

use bsw_time::Instant;

pub use diagnostics::{
    DiagnosticCore, DiagnosticResponse, DiagnosticTransport, DIAG_RESPONSE_CAPACITY,
};
pub use io::{IoError, IoModel, IoSnapshot, ADC_MAX, PWM_MAX_PERMILLE};

/// Platform-neutral production composition used unchanged on all three targets.
///
/// Platform applications add only their clock, I/O, transport, and storage
/// adapters around this owner. Diagnostic state, lifecycle state, and the I/O
/// model therefore cannot silently diverge by target.
pub struct ProductionComposition {
    diagnostics: DiagnosticCore,
    io: IoModel,
    running: bool,
    last_cycle: Instant,
}

impl ProductionComposition {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            diagnostics: DiagnosticCore::new(),
            io: IoModel::new(),
            running: false,
            last_cycle: Instant::from_nanos(0),
        }
    }

    pub fn start(&mut self, now: Instant) {
        self.running = true;
        self.last_cycle = now;
    }

    pub fn shutdown(&mut self, now: Instant) {
        self.running = false;
        self.last_cycle = now;
    }

    #[must_use]
    pub const fn is_running(&self) -> bool {
        self.running
    }

    #[must_use]
    pub const fn last_cycle(&self) -> Instant {
        self.last_cycle
    }

    pub fn cycle_io(&mut self, raw: u16, now: Instant) -> Result<IoSnapshot, IoError> {
        self.last_cycle = now;
        self.io.drive_adc(raw)
    }

    pub fn set_output(&mut self, high: bool, now: Instant) {
        self.last_cycle = now;
        self.io.set_output(high);
    }

    #[must_use]
    pub const fn io_snapshot(&self) -> IoSnapshot {
        self.io.snapshot()
    }

    pub fn diagnostics_mut(&mut self) -> &mut DiagnosticCore {
        &mut self.diagnostics
    }

    #[must_use]
    pub const fn diagnostics(&self) -> &DiagnosticCore {
        &self.diagnostics
    }
}

impl Default for ProductionComposition {
    fn default() -> Self {
        Self::new()
    }
}

/// Compatibility name retained for downstream code while all production
/// entrypoints use ProductionComposition explicitly.
pub type ReferenceCore = ProductionComposition;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lifecycle_and_io_use_only_injected_time() {
        let mut app = ReferenceCore::new();
        let start = Instant::from_nanos(u64::MAX - 4);
        app.start(start);
        assert!(app.is_running());
        assert_eq!(
            app.cycle_io(
                ADC_MAX,
                start.wrapping_add(bsw_time::Duration::from_nanos(8))
            )
            .unwrap()
            .pwm_permille,
            PWM_MAX_PERMILLE
        );
        app.shutdown(Instant::from_nanos(7));
        assert!(!app.is_running());
        assert_eq!(app.last_cycle(), Instant::from_nanos(7));
    }
}
