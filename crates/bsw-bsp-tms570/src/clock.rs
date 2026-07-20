//! Exact clock policy for LAUNCHXL2-570LC43.
//!
//! The frequency model follows SPNU563A chapters 2, 14, and 15. The bounded
//! PLL acceptance policy implements the three failure signatures described by
//! SPNZ232B SSWF021#45: CSVSTAT without a clean slip state is insufficient,
//! and a clean status still requires a DCC frequency pass before PLL1 may be
//! selected. This module contains no MMIO; target register access is added only
//! after these state and arithmetic invariants are independently testable.

use crate::board::MCU_OSCILLATOR_HZ;

/// Minimum bounded attempt count recommended by SPNZ232B SSWF021#45.
pub const PLL_STARTUP_ATTEMPTS: u8 = 5;

const MIN_OSCILLATOR_HZ: u32 = 5_000_000;
const MAX_OSCILLATOR_HZ: u32 = 20_000_000;
const MIN_PLL_INTERNAL_HZ: u32 = 1_000_000;
const MIN_PLL_VCO_HZ: u32 = 150_000_000;
const MAX_PLL_VCO_HZ: u32 = 550_000_000;
const MAX_GCLK_HZ: u32 = 300_000_000;
const MAX_DCC_SEED: u32 = 0x000f_ffff;

/// Exact non-modulated PLL1 configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PllConfig {
    /// Main-oscillator input frequency.
    pub oscillator_hz: u32,
    /// Reference divider NR, 1 through 64.
    pub reference_divider: u8,
    /// Feedback multiplier NF, 1 through 256.
    pub multiplier: u16,
    /// Output divider OD, 1 through 8.
    pub output_divider: u8,
    /// Final PLL divider R, 1 through 32.
    pub final_divider: u8,
}

/// Rejected clock-tree invariant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockConfigError {
    /// OSCIN is outside the documented 5-to-20-MHz range.
    OscillatorOutOfRange,
    /// A PLL divider or multiplier is outside its encoded range.
    PllFieldOutOfRange,
    /// The divided PLL reference clock is not an integer or is below 1 MHz.
    PllReferenceOutOfRange,
    /// The PLL VCO is outside the documented 150-to-550-MHz range.
    PllVcoOutOfRange,
    /// The selected GCLK exceeds the LC4357 limit.
    GclkOutOfRange,
    /// A synchronous-domain divider violates the LC4357 clock relationships.
    DomainRatioInvalid,
    /// DCC seed values cannot express the requested measurement window.
    DccWindowInvalid,
}

impl PllConfig {
    /// Validate the fields and return the final PLL clock.
    pub fn output_hz(self) -> Result<u32, ClockConfigError> {
        if !(MIN_OSCILLATOR_HZ..=MAX_OSCILLATOR_HZ).contains(&self.oscillator_hz) {
            return Err(ClockConfigError::OscillatorOutOfRange);
        }
        if !(1..=64).contains(&self.reference_divider)
            || !(1..=256).contains(&self.multiplier)
            || !(1..=8).contains(&self.output_divider)
            || !(1..=32).contains(&self.final_divider)
        {
            return Err(ClockConfigError::PllFieldOutOfRange);
        }

        let reference_divider = u32::from(self.reference_divider);
        if !self.oscillator_hz.is_multiple_of(reference_divider) {
            return Err(ClockConfigError::PllReferenceOutOfRange);
        }
        let internal_hz = self.oscillator_hz / reference_divider;
        if internal_hz < MIN_PLL_INTERNAL_HZ {
            return Err(ClockConfigError::PllReferenceOutOfRange);
        }

        let vco_hz = u64::from(internal_hz) * u64::from(self.multiplier);
        if !(u64::from(MIN_PLL_VCO_HZ)..=u64::from(MAX_PLL_VCO_HZ)).contains(&vco_hz) {
            return Err(ClockConfigError::PllVcoOutOfRange);
        }
        let total_divider = u64::from(self.output_divider) * u64::from(self.final_divider);
        if !vco_hz.is_multiple_of(total_divider) {
            return Err(ClockConfigError::PllFieldOutOfRange);
        }
        let output_hz = vco_hz / total_divider;
        if output_hz > u64::from(MAX_GCLK_HZ) {
            return Err(ClockConfigError::GclkOutOfRange);
        }
        u32::try_from(output_hz).map_err(|_| ClockConfigError::GclkOutOfRange)
    }

    /// PLLMUL field value, including the eight fractional encoding bits.
    pub const fn multiplier_field(self) -> u16 {
        (self.multiplier - 1) << 8
    }

    /// PLLCTL1 value used while locking with the R divider held at 32.
    pub const fn pllctl1_lock_value(self) -> u32 {
        self.pllctl1_value(32)
    }

    /// PLLCTL1 value after a successful DCC measurement.
    pub const fn pllctl1_final_value(self) -> u32 {
        self.pllctl1_value(self.final_divider)
    }

    const fn pllctl1_value(self, divider: u8) -> u32 {
        const BYPASS_ON_SLIP: u32 = 0x2000_0000;
        BYPASS_ON_SLIP
            | ((divider as u32 - 1) << 24)
            | ((self.reference_divider as u32 - 1) << 16)
            | self.multiplier_field() as u32
    }

    /// Non-modulated PLLCTL2 value with the exact OD and conservative filter
    /// fields selected by the LC4357 HALCoGen device configuration.
    pub const fn pllctl2_value(self) -> u32 {
        const SPREADING_RATE: u32 = 255;
        const BANDWIDTH_ADJUSTMENT: u32 = 7;
        const SPREADING_AMOUNT: u32 = 61;
        (SPREADING_RATE << 22)
            | (BANDWIDTH_ADJUSTMENT << 12)
            | ((self.output_divider as u32 - 1) << 9)
            | SPREADING_AMOUNT
    }
}

/// Exact synchronous clock-domain divisors and derived frequencies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockPlan {
    /// PLL1 settings used as the GCLK source only after acceptance.
    pub pll1: PllConfig,
    /// GCLK-to-HCLK integer divider.
    pub hclk_divider: u8,
    /// HCLK-to-VCLK integer divider.
    pub vclk_divider: u8,
    /// HCLK-to-VCLK2 integer divider.
    pub vclk2_divider: u8,
    /// HCLK-to-VCLK3 integer divider.
    pub vclk3_divider: u8,
}

/// Derived frequencies after a clock plan passes all checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockFrequencies {
    /// CPU clock.
    pub gclk_hz: u32,
    /// System interconnect clock.
    pub hclk_hz: u32,
    /// Primary peripheral clock.
    pub vclk_hz: u32,
    /// Secondary synchronous peripheral clock.
    pub vclk2_hz: u32,
    /// Third synchronous peripheral clock.
    pub vclk3_hz: u32,
    /// RTI clock, sourced from VCLK in this plan.
    pub rti_hz: u32,
}

impl ClockPlan {
    /// Validate domain relationships, including the GCM#58 workaround.
    pub fn frequencies(self) -> Result<ClockFrequencies, ClockConfigError> {
        let gclk_hz = self.pll1.output_hz()?;
        if !(1..=4).contains(&self.hclk_divider)
            || !(1..=16).contains(&self.vclk_divider)
            || !(1..=16).contains(&self.vclk2_divider)
            || !(1..=16).contains(&self.vclk3_divider)
        {
            return Err(ClockConfigError::DomainRatioInvalid);
        }

        let hclk_hz = gclk_hz / u32::from(self.hclk_divider);
        let primary_peripheral_hz = hclk_hz / u32::from(self.vclk_divider);
        let secondary_peripheral_hz = hclk_hz / u32::from(self.vclk2_divider);
        let third_peripheral_hz = hclk_hz / u32::from(self.vclk3_divider);
        if gclk_hz % u32::from(self.hclk_divider) != 0
            || hclk_hz % u32::from(self.vclk_divider) != 0
            || hclk_hz % u32::from(self.vclk2_divider) != 0
            || hclk_hz % u32::from(self.vclk3_divider) != 0
            || secondary_peripheral_hz % primary_peripheral_hz != 0
            || self.vclk_divider > 3
            || self.vclk2_divider > 3
        {
            return Err(ClockConfigError::DomainRatioInvalid);
        }

        Ok(ClockFrequencies {
            gclk_hz,
            hclk_hz,
            vclk_hz: primary_peripheral_hz,
            vclk2_hz: secondary_peripheral_hz,
            vclk3_hz: third_peripheral_hz,
            rti_hz: primary_peripheral_hz,
        })
    }
}

/// Exact production-oriented clock plan for the proven board.
pub const LAUNCHXL2_CLOCK_PLAN: ClockPlan = ClockPlan {
    pll1: PllConfig {
        oscillator_hz: MCU_OSCILLATOR_HZ,
        reference_divider: 8,
        multiplier: 150,
        output_divider: 1,
        final_divider: 1,
    },
    hclk_divider: 2,
    vclk_divider: 2,
    vclk2_divider: 2,
    vclk3_divider: 2,
};

/// Single-shot DCC tolerance window with OSCIN as clock0 and PLL1 as clock1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DccWindow {
    /// Reference-clock cycles before the valid window opens.
    pub count0_seed: u32,
    /// Reference-clock cycles in the acceptance window.
    pub valid0_seed: u16,
    /// Measured-clock cycles expected inside the reference window.
    pub count1_seed: u32,
}

impl DccWindow {
    /// Validate the hardware seed limits.
    pub const fn validate(self) -> Result<(), ClockConfigError> {
        if self.count0_seed == 0
            || self.count0_seed > MAX_DCC_SEED
            || self.valid0_seed < 4
            || self.count1_seed == 0
            || self.count1_seed > MAX_DCC_SEED
        {
            return Err(ClockConfigError::DccWindowInvalid);
        }
        Ok(())
    }

    /// Whether `measured_hz` makes counter1 expire no earlier than counter0
    /// and no later than the end of valid0.
    pub fn accepts(self, reference_hz: u32, measured_hz: u32) -> bool {
        if self.validate().is_err() || reference_hz == 0 || measured_hz == 0 {
            return false;
        }
        let count1_reference = u64::from(self.count1_seed) * u64::from(reference_hz);
        let earliest = u64::from(self.count0_seed) * u64::from(measured_hz);
        let latest =
            (u64::from(self.count0_seed) + u64::from(self.valid0_seed)) * u64::from(measured_hz);
        count1_reference >= earliest && count1_reference <= latest
    }
}

/// Approximately +/-0.5% DCC window centered on 300 MHz against 16-MHz OSCIN.
pub const PLL1_DCC_WINDOW: DccWindow = DccWindow {
    count0_seed: 1_600,
    valid0_seed: 16,
    count1_seed: 30_150,
};

/// DCC completion state observed for one PLL attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DccOutcome {
    /// The single-shot measurement completed inside the configured window.
    Done,
    /// DCC reported a frequency error.
    Error,
    /// The bounded poll expired without DONE or ERR.
    Timeout,
}

/// All observations required before PLL1 may source GCLK.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PllObservation {
    /// PLL1 bit in CSVSTAT.
    pub clock_source_valid: bool,
    /// PLL1 slip state from ESM/global status.
    pub slip_detected: bool,
    /// Independent DCC frequency result.
    pub dcc: DccOutcome,
}

impl PllObservation {
    const fn accepted(self) -> bool {
        self.clock_source_valid && !self.slip_detected && matches!(self.dcc, DccOutcome::Done)
    }
}

/// Next action from the bounded PLL startup state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PllDecision {
    /// PLL1 passed all three checks and may now be selected.
    SelectPll1,
    /// Disable, clear status, and start the numbered next attempt.
    Retry { next_attempt: u8 },
    /// Keep OSCIN selected and enter the platform fault path.
    FailClosed { attempts: u8 },
}

/// Invalid use of the PLL attempt state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PllStateError {
    /// An attempt is already waiting for observations.
    AttemptInProgress,
    /// No attempt is waiting for observations.
    NoAttemptInProgress,
    /// A terminal decision has already been made.
    Resolved,
}

/// Bounded SSWF021#45 attempt controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PllStartup {
    attempts_started: u8,
    awaiting_observation: bool,
    resolved: bool,
}

impl PllStartup {
    /// Create a fresh power-on PLL controller.
    pub const fn new() -> Self {
        Self {
            attempts_started: 0,
            awaiting_observation: false,
            resolved: false,
        }
    }

    /// Start one attempt after the caller has selected OSCIN, disabled PLL1,
    /// and cleared its slip state.
    pub fn begin_attempt(&mut self) -> Result<u8, PllStateError> {
        if self.resolved {
            return Err(PllStateError::Resolved);
        }
        if self.awaiting_observation {
            return Err(PllStateError::AttemptInProgress);
        }
        self.attempts_started += 1;
        self.awaiting_observation = true;
        Ok(self.attempts_started)
    }

    /// Classify CSVSTAT, slip, and DCC only after a bounded target poll.
    pub fn observe(&mut self, observation: PllObservation) -> Result<PllDecision, PllStateError> {
        if self.resolved {
            return Err(PllStateError::Resolved);
        }
        if !self.awaiting_observation {
            return Err(PllStateError::NoAttemptInProgress);
        }
        self.awaiting_observation = false;

        if observation.accepted() {
            self.resolved = true;
            return Ok(PllDecision::SelectPll1);
        }
        if self.attempts_started >= PLL_STARTUP_ATTEMPTS {
            self.resolved = true;
            return Ok(PllDecision::FailClosed {
                attempts: self.attempts_started,
            });
        }
        Ok(PllDecision::Retry {
            next_attempt: self.attempts_started + 1,
        })
    }
}

impl Default for PllStartup {
    fn default() -> Self {
        Self::new()
    }
}

const _: () = {
    assert!(PLL_STARTUP_ATTEMPTS == 5);
    assert!(PLL1_DCC_WINDOW.validate().is_ok());
};

#[cfg(test)]
mod tests {
    use super::*;

    const PASS: PllObservation = PllObservation {
        clock_source_valid: true,
        slip_detected: false,
        dcc: DccOutcome::Done,
    };

    #[test]
    fn exact_board_plan_derives_production_frequencies() {
        let clocks = LAUNCHXL2_CLOCK_PLAN.frequencies().unwrap();
        assert_eq!(clocks.gclk_hz, 300_000_000);
        assert_eq!(clocks.hclk_hz, 150_000_000);
        assert_eq!(clocks.vclk_hz, 75_000_000);
        assert_eq!(clocks.vclk2_hz, clocks.vclk_hz);
        assert_eq!(clocks.vclk3_hz, 75_000_000);
        assert_eq!(clocks.rti_hz, 75_000_000);
    }

    #[test]
    fn exact_pll_fields_match_independent_halco_device_model() {
        let pll = LAUNCHXL2_CLOCK_PLAN.pll1;
        assert_eq!(pll.multiplier_field(), 0x9500);
        assert_eq!(pll.pllctl1_lock_value(), 0x3f07_9500);
        assert_eq!(pll.pllctl1_final_value(), 0x2007_9500);
        assert_eq!(pll.pllctl2_value(), 0x3fc0_703d);
    }

    #[test]
    fn clock_plan_rejects_gcm58_slow_vclk_ratio() {
        let mut invalid = LAUNCHXL2_CLOCK_PLAN;
        invalid.vclk_divider = 4;
        assert_eq!(
            invalid.frequencies(),
            Err(ClockConfigError::DomainRatioInvalid)
        );
    }

    #[test]
    fn pll_model_rejects_invalid_reference_and_vco() {
        let mut invalid = LAUNCHXL2_CLOCK_PLAN.pll1;
        invalid.reference_divider = 64;
        assert_eq!(
            invalid.output_hz(),
            Err(ClockConfigError::PllReferenceOutOfRange)
        );
        invalid = LAUNCHXL2_CLOCK_PLAN.pll1;
        invalid.multiplier = 1;
        assert_eq!(invalid.output_hz(), Err(ClockConfigError::PllVcoOutOfRange));
    }

    #[test]
    fn dcc_window_accepts_only_the_declared_frequency_band() {
        assert!(PLL1_DCC_WINDOW.accepts(MCU_OSCILLATOR_HZ, 300_000_000));
        assert!(PLL1_DCC_WINDOW.accepts(MCU_OSCILLATOR_HZ, 299_000_000));
        assert!(PLL1_DCC_WINDOW.accepts(MCU_OSCILLATOR_HZ, 301_000_000));
        assert!(!PLL1_DCC_WINDOW.accepts(MCU_OSCILLATOR_HZ, 298_000_000));
        assert!(!PLL1_DCC_WINDOW.accepts(MCU_OSCILLATOR_HZ, 302_000_000));
    }

    #[test]
    fn all_three_erratum_failure_signatures_retry() {
        let failures = [
            PllObservation {
                clock_source_valid: true,
                slip_detected: true,
                dcc: DccOutcome::Done,
            },
            PllObservation {
                clock_source_valid: false,
                slip_detected: true,
                dcc: DccOutcome::Error,
            },
            PllObservation {
                clock_source_valid: true,
                slip_detected: false,
                dcc: DccOutcome::Error,
            },
        ];

        for failure in failures {
            let mut startup = PllStartup::new();
            assert_eq!(startup.begin_attempt(), Ok(1));
            assert_eq!(
                startup.observe(failure),
                Ok(PllDecision::Retry { next_attempt: 2 })
            );
        }
    }

    #[test]
    fn clean_status_and_dcc_pass_select_pll() {
        let mut startup = PllStartup::new();
        assert_eq!(startup.begin_attempt(), Ok(1));
        assert_eq!(startup.observe(PASS), Ok(PllDecision::SelectPll1));
        assert_eq!(startup.begin_attempt(), Err(PllStateError::Resolved));
    }

    #[test]
    fn fifth_failed_attempt_is_fail_closed() {
        let timeout = PllObservation {
            clock_source_valid: false,
            slip_detected: false,
            dcc: DccOutcome::Timeout,
        };
        let mut startup = PllStartup::new();
        for attempt in 1..PLL_STARTUP_ATTEMPTS {
            assert_eq!(startup.begin_attempt(), Ok(attempt));
            assert_eq!(
                startup.observe(timeout),
                Ok(PllDecision::Retry {
                    next_attempt: attempt + 1,
                })
            );
        }
        assert_eq!(startup.begin_attempt(), Ok(PLL_STARTUP_ATTEMPTS));
        assert_eq!(
            startup.observe(timeout),
            Ok(PllDecision::FailClosed {
                attempts: PLL_STARTUP_ATTEMPTS,
            })
        );
    }

    #[test]
    fn state_machine_rejects_misordered_calls() {
        let mut startup = PllStartup::new();
        assert_eq!(
            startup.observe(PASS),
            Err(PllStateError::NoAttemptInProgress)
        );
        assert_eq!(startup.begin_attempt(), Ok(1));
        assert_eq!(
            startup.begin_attempt(),
            Err(PllStateError::AttemptInProgress)
        );
    }
}
