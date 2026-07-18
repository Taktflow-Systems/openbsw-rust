//! Shared PWM frequency/duty calculation and STM32 channel contract (G08).

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PwmError {
    ZeroFrequency,
    FrequencyTooHigh,
    DutyOutOfRange,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PwmRegisters {
    pub prescaler: u16,
    pub auto_reload: u16,
    pub compare: u16,
}

pub fn calculate(
    timer_hz: u32,
    frequency_hz: u32,
    duty_permille: u16,
) -> Result<PwmRegisters, PwmError> {
    if frequency_hz == 0 {
        return Err(PwmError::ZeroFrequency);
    }
    if duty_permille > 1000 {
        return Err(PwmError::DutyOutOfRange);
    }
    if frequency_hz > timer_hz {
        return Err(PwmError::FrequencyTooHigh);
    }
    let total_ticks = timer_hz / frequency_hz;
    let prescaler = total_ticks.saturating_sub(1) / 65_536;
    let period = total_ticks / (prescaler + 1);
    if period == 0 || prescaler > u32::from(u16::MAX) {
        return Err(PwmError::FrequencyTooHigh);
    }
    let auto_reload = period.saturating_sub(1).min(u32::from(u16::MAX)) as u16;
    let compare = ((u32::from(auto_reload) + 1) * u32::from(duty_permille) / 1000) as u16;
    Ok(PwmRegisters {
        prescaler: prescaler as u16,
        auto_reload,
        compare,
    })
}

pub trait PwmChannel {
    fn configure(&mut self, frequency_hz: u32, duty_permille: u16) -> Result<(), PwmError>;
    fn duty_permille(&self) -> u16;
}

/// Board PWM channel: TIM3_CH1/PA6 on F413, TIM2_CH1/PA0 on G474.
#[cfg(all(target_arch = "arm", any(feature = "stm32f413", feature = "stm32g474")))]
pub struct Stm32PwmChannel {
    duty_permille: u16,
}

#[cfg(all(target_arch = "arm", any(feature = "stm32f413", feature = "stm32g474")))]
impl Stm32PwmChannel {
    const TIMER_HZ: u32 = crate::board::ACTIVE.core_clock_hz;

    /// Consume the unique PWM token and configure the board pin alternate function.
    pub fn new(_token: crate::board::Pwm) -> Self {
        #[cfg(feature = "stm32f413")]
        const GPIO: usize = 0x4002_0000;
        #[cfg(feature = "stm32g474")]
        const GPIO: usize = 0x4800_0000;
        #[cfg(feature = "stm32f413")]
        const RCC_GPIO_ENR: usize = 0x4002_3830;
        #[cfg(feature = "stm32g474")]
        const RCC_GPIO_ENR: usize = 0x4002_104C;
        #[cfg(feature = "stm32f413")]
        const RCC_TIM_ENR: usize = 0x4002_3840;
        #[cfg(feature = "stm32g474")]
        const RCC_TIM_ENR: usize = 0x4002_1058;
        #[cfg(feature = "stm32f413")]
        const TIM_ENABLE: u32 = 1 << 1;
        #[cfg(feature = "stm32g474")]
        const TIM_ENABLE: u32 = 1 << 1;
        #[cfg(feature = "stm32f413")]
        const PIN: usize = 6;
        #[cfg(feature = "stm32g474")]
        const PIN: usize = 6;
        #[cfg(feature = "stm32f413")]
        const AF: u32 = 2;
        #[cfg(feature = "stm32g474")]
        const AF: u32 = 2;
        // SAFETY: the MCU feature selects valid RCC/GPIO registers and the
        // consumed token provides unique ownership of this board channel.
        unsafe {
            modify32(RCC_GPIO_ENR, 0, 1);
            modify32(RCC_TIM_ENR, 0, TIM_ENABLE);
            modify32(GPIO, 0b11 << (PIN * 2), 0b10 << (PIN * 2));
            modify32(GPIO + 0x20, 0b1111 << (PIN * 4), AF << (PIN * 4));
        }
        Self { duty_permille: 0 }
    }
}

#[cfg(all(target_arch = "arm", any(feature = "stm32f413", feature = "stm32g474")))]
impl PwmChannel for Stm32PwmChannel {
    fn configure(&mut self, frequency_hz: u32, duty_permille: u16) -> Result<(), PwmError> {
        const TIM: usize = 0x4000_0400;
        let values = calculate(Self::TIMER_HZ, frequency_hz, duty_permille)?;
        // SAFETY: unique token ownership prevents concurrent TIM channel access.
        unsafe {
            write32(TIM, 0);
            write32(TIM + 0x28, u32::from(values.prescaler));
            write32(TIM + 0x2C, u32::from(values.auto_reload));
            write32(TIM + 0x34, u32::from(values.compare));
            write32(TIM + 0x18, (0b110 << 4) | (1 << 3));
            write32(TIM + 0x20, 1);
            write32(TIM + 0x14, 1);
            write32(TIM, (1 << 7) | 1);
        }
        self.duty_permille = duty_permille;
        Ok(())
    }

    fn duty_permille(&self) -> u16 {
        self.duty_permille
    }
}

#[cfg(all(target_arch = "arm", any(feature = "stm32f413", feature = "stm32g474")))]
unsafe fn write32(address: usize, value: u32) {
    // SAFETY: caller provides an aligned register selected for the active MCU.
    unsafe { crate::mmio::write(address as *mut u32, value) };
}

#[cfg(all(target_arch = "arm", any(feature = "stm32f413", feature = "stm32g474")))]
unsafe fn modify32(address: usize, clear: u32, set: u32) {
    // SAFETY: caller provides an aligned register selected for the active MCU.
    let value = unsafe { crate::mmio::read(address as *const u32) };
    // SAFETY: same register and ownership as above.
    unsafe { write32(address, (value & !clear) | set) };
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn both_board_clocks_cover_duty_and_frequency_boundaries() {
        for clock in [96_000_000, 170_000_000] {
            assert_eq!(calculate(clock, 1_000, 0).unwrap().compare, 0);
            let full = calculate(clock, 1_000, 1000).unwrap();
            assert_eq!(full.compare, full.auto_reload + 1);
            assert_eq!(calculate(clock, 0, 1), Err(PwmError::ZeroFrequency));
            assert_eq!(calculate(clock, 1_000, 1001), Err(PwmError::DutyOutOfRange));
        }
    }
}
