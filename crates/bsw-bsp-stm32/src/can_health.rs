//! Controller-independent CAN interrupt/error/recovery policy (G04).

use bsw_can::{ErrorStateTracker, TransceiverState};
use bsw_time::{Duration, Instant};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(clippy::struct_excessive_bools)]
pub struct CanInterruptFlags {
    pub rx: bool,
    pub tx_complete: bool,
    pub error_passive: bool,
    pub bus_off: bool,
    pub protocol_error: bool,
}

impl CanInterruptFlags {
    pub const fn from_fdcan_ir(raw: u32) -> Self {
        Self {
            rx: raw & (1 << 0) != 0,
            tx_complete: raw & (1 << 9) != 0,
            error_passive: raw & (1 << 23) != 0,
            bus_off: raw & (1 << 25) != 0,
            protocol_error: raw & ((1 << 22) | (1 << 28)) != 0,
        }
    }

    pub const fn from_bxcan(msr: u32, esr: u32) -> Self {
        Self {
            rx: false,
            tx_complete: msr & (1 << 8) != 0,
            error_passive: esr & (1 << 1) != 0,
            bus_off: esr & (1 << 2) != 0,
            protocol_error: esr & 0x70 != 0,
        }
    }
}

/// Minimal register seam used by host mock-register tests.
pub trait RegisterBank {
    fn read(&self, offset: usize) -> u32;
    fn write(&mut self, offset: usize, value: u32);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControllerKind {
    BxCan,
    FdCan,
}

pub struct CanHealth {
    tracker: ErrorStateTracker,
    recovery_delay: Duration,
    error_interrupts: u32,
    queue_overflows: u32,
}

impl CanHealth {
    pub const fn new(recovery_delay: Duration) -> Self {
        Self {
            tracker: ErrorStateTracker::new(),
            recovery_delay,
            error_interrupts: 0,
            queue_overflows: 0,
        }
    }

    pub fn handle(&mut self, flags: CanInterruptFlags, now: Instant) -> TransceiverState {
        if flags.protocol_error || flags.error_passive || flags.bus_off {
            self.error_interrupts = self.error_interrupts.saturating_add(1);
        }
        if flags.protocol_error {
            let _ = self.tracker.record_tx_error();
        }
        if flags.error_passive {
            while self.tracker.state() == TransceiverState::Active {
                let _ = self.tracker.record_tx_error();
            }
        }
        if flags.bus_off && self.tracker.state() != TransceiverState::BusOff {
            while self.tracker.state() != TransceiverState::BusOff {
                let _ = self.tracker.record_tx_error();
            }
            self.tracker.begin_recovery(now, self.recovery_delay);
        }
        self.tracker.state()
    }

    pub fn poll_recovery(&mut self, now: Instant) -> Option<TransceiverState> {
        self.tracker.poll_recovery(now)
    }

    pub fn observe_queue_drops(&mut self, total: u32) {
        self.queue_overflows = self.queue_overflows.max(total);
    }

    pub const fn state(&self) -> TransceiverState {
        self.tracker.state()
    }

    pub const fn error_interrupts(&self) -> u32 {
        self.error_interrupts
    }

    pub const fn queue_overflows(&self) -> u32 {
        self.queue_overflows
    }
}

/// Read and acknowledge one controller's interrupt flags through a mockable
/// register bank. Offsets are deliberately centralized here.
pub fn service_registers(bank: &mut dyn RegisterBank, kind: ControllerKind) -> CanInterruptFlags {
    match kind {
        ControllerKind::FdCan => {
            const IR: usize = 0x050;
            let raw = bank.read(IR);
            bank.write(IR, raw);
            CanInterruptFlags::from_fdcan_ir(raw)
        }
        ControllerKind::BxCan => {
            const MSR: usize = 0x004;
            const ESR: usize = 0x018;
            CanInterruptFlags::from_bxcan(bank.read(MSR), bank.read(ESR))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockRegisters {
        values: [u32; 32],
        acknowledged: u32,
    }

    impl RegisterBank for MockRegisters {
        fn read(&self, offset: usize) -> u32 {
            self.values[offset / 4]
        }
        fn write(&mut self, _offset: usize, value: u32) {
            self.acknowledged = value;
        }
    }

    #[test]
    fn fdcan_flags_are_decoded_and_w1c_acknowledged() {
        let raw = (1 << 0) | (1 << 23) | (1 << 25);
        let mut registers = MockRegisters {
            values: [0; 32],
            acknowledged: 0,
        };
        registers.values[0x50 / 4] = raw;
        let flags = service_registers(&mut registers, ControllerKind::FdCan);
        assert!(flags.rx && flags.error_passive && flags.bus_off);
        assert_eq!(registers.acknowledged, raw);
    }

    #[test]
    fn bus_off_recovery_and_overflow_boundaries_are_exact() {
        let delay = Duration::from_nanos(1_000);
        let mut health = CanHealth::new(delay);
        let start = Instant::from_nanos(10);
        assert_eq!(
            health.handle(
                CanInterruptFlags {
                    bus_off: true,
                    ..CanInterruptFlags::default()
                },
                start
            ),
            TransceiverState::BusOff
        );
        assert_eq!(health.poll_recovery(Instant::from_nanos(1_009)), None);
        assert_eq!(
            health.poll_recovery(Instant::from_nanos(1_010)),
            Some(TransceiverState::Active)
        );
        health.observe_queue_drops(3);
        health.observe_queue_drops(2);
        assert_eq!(health.queue_overflows(), 3);
    }
}
