//! DWT monotonic clock with remainder and wrap preservation (G05).

use core::cell::Cell;

use bsw_can::transceiver::SystemTimer;
use bsw_time::{Clock, Instant};
use cortex_m::peripheral::{DCB, DWT};

/// Host-testable conversion from a wrapping cycle counter to nanoseconds.
pub struct CycleAccumulator {
    cycles_per_second: Cell<u32>,
    last: Cell<u32>,
    nanoseconds: Cell<u64>,
    remainder: Cell<u64>,
}

impl CycleAccumulator {
    pub const fn new() -> Self {
        Self {
            cycles_per_second: Cell::new(0),
            last: Cell::new(0),
            nanoseconds: Cell::new(0),
            remainder: Cell::new(0),
        }
    }

    pub fn calibrate(&self, cycles_per_second: u32, raw: u32) {
        assert!(
            cycles_per_second >= 1_000_000,
            "core clock must be at least 1 MHz"
        );
        self.cycles_per_second.set(cycles_per_second);
        self.last.set(raw);
        self.nanoseconds.set(0);
        self.remainder.set(0);
    }

    pub fn update(&self, raw: u32) -> Instant {
        let frequency = self.cycles_per_second.get();
        assert!(frequency != 0, "cycle accumulator is not calibrated");
        let elapsed = raw.wrapping_sub(self.last.replace(raw));
        let scaled = u64::from(elapsed) * 1_000_000_000 + self.remainder.get();
        let nanos = scaled / u64::from(frequency);
        self.remainder.set(scaled % u64::from(frequency));
        self.nanoseconds
            .set(self.nanoseconds.get().wrapping_add(nanos));
        Instant::from_nanos(self.nanoseconds.get())
    }

    pub fn now(&self) -> Instant {
        Instant::from_nanos(self.nanoseconds.get())
    }
}

impl Default for CycleAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

pub struct DwtTimer {
    accumulator: CycleAccumulator,
}

impl DwtTimer {
    pub const fn from_token(_token: crate::board::Timer) -> Self {
        Self::new()
    }

    pub const fn new() -> Self {
        Self {
            accumulator: CycleAccumulator::new(),
        }
    }

    pub fn init(&mut self, dcb: &mut DCB, dwt: &mut DWT, sys_clock_hz: u32) {
        dcb.enable_trace();
        DWT::unlock();
        dwt.enable_cycle_counter();
        self.accumulator.calibrate(sys_clock_hz, DWT::cycle_count());
    }

    pub fn update(&self) -> Instant {
        self.accumulator.update(DWT::cycle_count())
    }

    pub fn system_time_us_64(&self) -> u64 {
        self.update().as_nanos() / 1_000
    }

    pub fn freq_mhz(&self) -> u32 {
        self.accumulator.cycles_per_second.get() / 1_000_000
    }
}

impl Default for DwtTimer {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for DwtTimer {
    fn now(&self) -> Instant {
        self.update()
    }
}

impl SystemTimer for DwtTimer {
    fn system_time_us(&self) -> u32 {
        self.system_time_us_64() as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn remainder_accumulates_sub_nanosecond_fractions() {
        let clock = CycleAccumulator::new();
        clock.calibrate(170_000_000, 0);
        for raw in 1..=170 {
            clock.update(raw);
        }
        assert_eq!(clock.now(), Instant::from_nanos(1_000));
    }

    #[test]
    fn raw_counter_wrap_is_monotonic() {
        let clock = CycleAccumulator::new();
        clock.calibrate(1_000_000, u32::MAX - 4);
        assert_eq!(clock.update(5), Instant::from_nanos(10_000));
    }
}
