//! Deterministic ADC/PWM/GPIO application policy.

pub const ADC_MAX: u16 = 4_095;
pub const PWM_MAX_PERMILLE: u16 = 1_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IoError {
    AdcOutOfRange,
    PwmOutOfRange,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IoSnapshot {
    pub adc: u16,
    pub vehicle_speed: u16,
    pub pwm_permille: u16,
    pub output: bool,
}

#[derive(Debug, Default)]
pub struct IoModel {
    adc: u16,
    pwm_permille: u16,
    output: bool,
}

impl IoModel {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            adc: 0,
            pwm_permille: 0,
            output: false,
        }
    }

    pub fn drive_adc(&mut self, raw: u16) -> Result<IoSnapshot, IoError> {
        if raw > ADC_MAX {
            return Err(IoError::AdcOutOfRange);
        }
        self.adc = raw;
        self.pwm_permille =
            ((u32::from(raw) * u32::from(PWM_MAX_PERMILLE)) / u32::from(ADC_MAX)) as u16;
        Ok(self.snapshot())
    }

    pub fn set_pwm(&mut self, duty_permille: u16) -> Result<(), IoError> {
        if duty_permille > PWM_MAX_PERMILLE {
            return Err(IoError::PwmOutOfRange);
        }
        self.pwm_permille = duty_permille;
        Ok(())
    }

    pub fn set_output(&mut self, high: bool) {
        self.output = high;
    }

    #[must_use]
    pub const fn snapshot(&self) -> IoSnapshot {
        IoSnapshot {
            adc: self.adc,
            vehicle_speed: self.adc,
            pwm_permille: self.pwm_permille,
            output: self.output,
        }
    }
}
