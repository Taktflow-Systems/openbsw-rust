//! RTI counter-0 monotonic clock for TMS570LC4357.
//!
//! Counter 0 is clocked from the board plan's 75-MHz RTICLK and divided to
//! exactly 1 MHz. The 32-bit free-running counter therefore wraps after about
//! 71.6 minutes. [`RtiAccumulator`] extends that counter in software; callers
//! must sample at least once per hardware wrap because no software algorithm
//! can infer two or more unobserved wraps from a single 32-bit sample.

use core::cell::Cell;

#[cfg(target_arch = "arm")]
use bsw_can::transceiver::SystemTimer;
#[cfg(target_arch = "arm")]
use bsw_time::Clock;
use bsw_time::Instant;

#[cfg(target_arch = "arm")]
use crate::board;
use crate::device::RTI_DWWD_REGISTERS;

/// Frequency exposed by the production RTI counter.
pub const RTI_TICK_HZ: u32 = 1_000_000;
/// Maximum interval between observations before a wrap becomes ambiguous.
pub const MAX_UNAMBIGUOUS_SAMPLE_MICROS: u64 = 1_u64 << 32;

#[cfg(target_arch = "arm")]
const GCTRL: usize = 0x00;
#[cfg(target_arch = "arm")]
const TBCTRL: usize = 0x04;
#[cfg(target_arch = "arm")]
const FRC0: usize = 0x10;
#[cfg(target_arch = "arm")]
const UC0: usize = 0x14;
const CPUC0: usize = 0x18;
#[cfg(target_arch = "arm")]
const CNT0EN: u32 = 1;

/// Invalid RTI clock or prescaler selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtiConfigError {
    /// The input cannot be divided to the exact production tick rate.
    NonIntegralMicrosecondTick,
    /// The resulting prescaler would be zero or exceed the register.
    PrescalerOutOfRange,
}

/// Validated counter-0 clock configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtiConfig {
    input_hz: u32,
    cpuc0: u32,
}

impl RtiConfig {
    /// Derive the exact 1-MHz counter configuration from RTICLK.
    pub const fn microsecond_counter(input_hz: u32) -> Result<Self, RtiConfigError> {
        if !input_hz.is_multiple_of(RTI_TICK_HZ) {
            return Err(RtiConfigError::NonIntegralMicrosecondTick);
        }
        let divider = input_hz / RTI_TICK_HZ;
        if divider < 2 {
            return Err(RtiConfigError::PrescalerOutOfRange);
        }
        Ok(Self {
            input_hz,
            cpuc0: divider - 1,
        })
    }

    /// RTICLK supplied to counter 0.
    pub const fn input_hz(self) -> u32 {
        self.input_hz
    }

    /// Value written to the counter-0 prescaler register.
    pub const fn cpuc0(self) -> u32 {
        self.cpuc0
    }

    /// Resulting free-running counter frequency.
    pub const fn counter_hz(self) -> u32 {
        self.input_hz / (self.cpuc0 + 1)
    }
}

/// Exact RTI setup derived from the feature-selected board clock plan.
pub const LAUNCHXL2_RTI_CONFIG: RtiConfig = {
    match RtiConfig::microsecond_counter(75_000_000) {
        Ok(config) => config,
        Err(_) => panic!("board RTI clock must produce exact microseconds"),
    }
};

/// Coherent counter-0 observation.
///
/// The LC4357 latches UC0 when FRC0 is read, so the hardware adapter always
/// reads these fields in that order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtiSnapshot {
    /// Free-running 1-MHz tick count.
    pub frc0: u32,
    /// Prescaler/up-counter value latched with `frc0`.
    pub uc0: u32,
}

/// Host-testable extension of the wrapping 32-bit RTI counter.
pub struct RtiAccumulator {
    calibrated: Cell<bool>,
    last: Cell<u32>,
    microseconds: Cell<u64>,
}

impl RtiAccumulator {
    /// Create an accumulator which must be calibrated before use.
    pub const fn new() -> Self {
        Self {
            calibrated: Cell::new(false),
            last: Cell::new(0),
            microseconds: Cell::new(0),
        }
    }

    /// Establish the initial raw counter observation and zero the epoch.
    pub fn calibrate(&self, raw: u32) {
        self.last.set(raw);
        self.microseconds.set(0);
        self.calibrated.set(true);
    }

    /// Extend one raw sample into the monotonic software epoch.
    ///
    /// Samples must be separated by less than
    /// [`MAX_UNAMBIGUOUS_SAMPLE_MICROS`].
    pub fn update(&self, raw: u32) -> Instant {
        assert!(self.calibrated.get(), "RTI accumulator is not calibrated");
        let elapsed = raw.wrapping_sub(self.last.replace(raw));
        let micros = self.microseconds.get().wrapping_add(u64::from(elapsed));
        self.microseconds.set(micros);
        Instant::from_nanos(micros.wrapping_mul(1_000))
    }

    /// Last extended instant without sampling hardware.
    pub fn now(&self) -> Instant {
        Instant::from_nanos(self.microseconds.get().wrapping_mul(1_000))
    }
}

impl Default for RtiAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

/// Single-owner hardware timer backed by RTI counter block 0.
#[cfg(target_arch = "arm")]
pub struct RtiTimer {
    _token: board::Rti,
    accumulator: RtiAccumulator,
}

#[cfg(target_arch = "arm")]
impl RtiTimer {
    /// Bind the unique board token without touching hardware.
    pub const fn from_token(token: board::Rti) -> Self {
        Self {
            _token: token,
            accumulator: RtiAccumulator::new(),
        }
    }

    /// Stop and initialize counter 0 as an exact 1-MHz free-running source.
    ///
    /// The clock plan must already be active. Counter 1 and every DWWD register
    /// are deliberately left untouched.
    pub fn init(&mut self) {
        let config = LAUNCHXL2_RTI_CONFIG;
        // SAFETY: the private board token makes this the sole RTI0 owner. The
        // offsets are LC4357 RTI registers; all writes are 32-bit aligned.
        unsafe {
            write_register(GCTRL, 0);
            write_register(TBCTRL, 0);
            write_register(FRC0, 0);
            write_register(UC0, 0);
            write_register(CPUC0, config.cpuc0());
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            write_register(GCTRL, CNT0EN);
            core::arch::asm!("dsb sy", "isb", options(nostack, preserves_flags));
        }
        self.accumulator.calibrate(self.snapshot().frc0);
    }

    /// Read FRC0 followed by the UC0 value latched by that read.
    pub fn snapshot(&self) -> RtiSnapshot {
        // SAFETY: ownership is held by this driver; both addresses are aligned
        // read-only observations in the exact LC4357 RTI register frame.
        unsafe {
            let frc0 = read_register(FRC0);
            let uc0 = read_register(UC0);
            RtiSnapshot { frc0, uc0 }
        }
    }

    fn update(&self) -> Instant {
        self.accumulator.update(self.snapshot().frc0)
    }
}

#[cfg(target_arch = "arm")]
impl Clock for RtiTimer {
    fn now(&self) -> Instant {
        self.update()
    }
}

#[cfg(target_arch = "arm")]
impl SystemTimer for RtiTimer {
    fn system_time_us(&self) -> u32 {
        (self.update().as_nanos() / 1_000) as u32
    }
}

#[cfg(target_arch = "arm")]
unsafe fn write_register(offset: usize, value: u32) {
    let address = RTI_DWWD_REGISTERS.start as usize + offset;
    // SAFETY: the caller proves that the exact aligned RTI register is
    // writable and uniquely owned by the driver.
    unsafe { core::ptr::write_volatile(address as *mut u32, value) };
}

#[cfg(target_arch = "arm")]
unsafe fn read_register(offset: usize) -> u32 {
    let address = RTI_DWWD_REGISTERS.start as usize + offset;
    // SAFETY: the caller proves that the exact aligned RTI register is valid
    // for a volatile observation.
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

const _: () = {
    assert!(LAUNCHXL2_RTI_CONFIG.input_hz() == 75_000_000);
    assert!(LAUNCHXL2_RTI_CONFIG.cpuc0() == 74);
    assert!(LAUNCHXL2_RTI_CONFIG.counter_hz() == RTI_TICK_HZ);
    assert!(RTI_DWWD_REGISTERS.contains_range(RTI_DWWD_REGISTERS.start + CPUC0 as u32, 4));
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_board_prescaler_produces_one_microsecond_ticks() {
        assert_eq!(LAUNCHXL2_RTI_CONFIG.input_hz(), 75_000_000);
        assert_eq!(LAUNCHXL2_RTI_CONFIG.cpuc0(), 74);
        assert_eq!(LAUNCHXL2_RTI_CONFIG.counter_hz(), 1_000_000);
    }

    #[test]
    fn invalid_prescaler_inputs_fail_closed() {
        assert_eq!(
            RtiConfig::microsecond_counter(75_000_001),
            Err(RtiConfigError::NonIntegralMicrosecondTick)
        );
        assert_eq!(
            RtiConfig::microsecond_counter(1_000_000),
            Err(RtiConfigError::PrescalerOutOfRange)
        );
    }

    #[test]
    fn accumulator_extends_raw_wrap_monotonically() {
        let accumulator = RtiAccumulator::new();
        accumulator.calibrate(u32::MAX - 4);
        assert_eq!(accumulator.update(5), Instant::from_nanos(10_000));
        assert_eq!(accumulator.update(105), Instant::from_nanos(110_000));
    }

    #[test]
    fn repeated_updates_preserve_exact_microseconds() {
        let accumulator = RtiAccumulator::new();
        accumulator.calibrate(100);
        for raw in 101..=1_100 {
            accumulator.update(raw);
        }
        assert_eq!(accumulator.now(), Instant::from_nanos(1_000_000));
    }
}
