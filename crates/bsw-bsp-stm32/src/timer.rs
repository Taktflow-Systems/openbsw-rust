//! DWT-based system timer — implements `SystemTimer` trait.
//!
//! Uses the ARM Cortex-M4 Data Watchpoint and Trace (DWT) cycle counter
//! to provide microsecond-resolution monotonic timing. Tracks a 64-bit
//! accumulator to handle 32-bit CYCCNT overflow.
//!
//! Maps to C++: `contribution/platforms/stm32/bsp/bspTimer/src/bsp/SystemTimer/SystemTimer.cpp`

use cortex_m::peripheral::{DCB, DWT};

use bsw_can::transceiver::SystemTimer;

/// DWT cycle-counter based system timer.
///
/// Must call [`DwtTimer::init`] once after clock setup to calibrate
/// the cycle-to-microsecond ratio.
pub struct DwtTimer {
    /// CPU frequency in MHz (e.g. 96 for F4, 170 for G4).
    freq_mhz: u32,
    /// Last raw CYCCNT reading (for overflow detection).
    last_cyccnt: u32,
    /// Accumulated microseconds (64-bit to avoid overflow).
    ticks_us: u64,
    /// Remainder cycles from integer division (prevents truncation loss).
    remainder_cycles: u32,
}

impl DwtTimer {
    /// Create an uninitialized timer. Call [`init`](Self::init) before use.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            freq_mhz: 0,
            last_cyccnt: 0,
            ticks_us: 0,
            remainder_cycles: 0,
        }
    }

    /// Initialize the DWT cycle counter.
    ///
    /// # Arguments
    /// * `dcb` — Debug Control Block peripheral (to enable TRCENA)
    /// * `dwt` — DWT peripheral (cycle counter)
    /// * `sys_clock_hz` — System core clock in Hz (e.g. `96_000_000`)
    ///
    /// # Safety
    /// Must be called once, before any other timer method, with interrupts
    /// disabled or before the scheduler starts.
    pub fn init(&mut self, dcb: &mut DCB, dwt: &mut DWT, sys_clock_hz: u32) {
        self.freq_mhz = sys_clock_hz / 1_000_000;
        assert!(self.freq_mhz > 0, "sys_clock_hz must be >= 1 MHz");

        // Enable the DWT cycle counter
        dcb.enable_trace();
        DWT::unlock();
        dwt.enable_cycle_counter();

        self.last_cyccnt = DWT::cycle_count();
        self.ticks_us = 0;
        self.remainder_cycles = 0;
    }

    /// Update the 64-bit microsecond accumulator from the 32-bit CYCCNT.
    ///
    /// Must be called periodically (at least once per CYCCNT overflow period,
    /// which is ~44s at 96 MHz or ~25s at 170 MHz). Typically called from
    /// `system_time_us()` and/or a periodic tick ISR.
    pub fn update(&mut self) {
        let now = DWT::cycle_count();
        let elapsed_cycles = now.wrapping_sub(self.last_cyccnt);
        self.last_cyccnt = now;
        // Accumulate cycles including remainder from previous call to prevent
        // integer division truncation in tight loops (e.g. 68 / 170 = 0).
        let total_cycles = elapsed_cycles + self.remainder_cycles;
        self.ticks_us += u64::from(total_cycles / self.freq_mhz);
        self.remainder_cycles = total_cycles % self.freq_mhz;
    }

    /// Return the 64-bit microsecond timestamp (full precision).
    pub fn system_time_us_64(&mut self) -> u64 {
        self.update();
        self.ticks_us
    }

    /// Return the system clock frequency in MHz.
    #[must_use]
    pub const fn freq_mhz(&self) -> u32 {
        self.freq_mhz
    }
}

impl SystemTimer for DwtTimer {
    /// Returns current system time in microseconds (lower 32 bits).
    ///
    /// Wraps every ~71 minutes. For longer durations use [`system_time_us_64`].
    fn system_time_us(&self) -> u32 {
        // NOTE: We cannot call update() here because the trait takes &self.
        // The caller must ensure update() is called periodically (e.g. in a
        // tick ISR or by using system_time_us_64 which takes &mut self).
        #[allow(clippy::cast_possible_truncation)]
        let us = self.ticks_us as u32;
        us
    }
}
