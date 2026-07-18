//! Shared ADC calibration/range/error contract (G09).

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdcError {
    NotCalibrated,
    CalibrationFailed,
    OutOfRange,
    HardwareTimeout,
}

/// Board ADC channel: ADC1_IN0/PA0 on F413, ADC1_IN1/PA0 on G474.
#[cfg(all(target_arch = "arm", any(feature = "stm32f413", feature = "stm32g474")))]
pub struct Stm32AdcChannel {
    calibrated: bool,
}

#[cfg(all(target_arch = "arm", any(feature = "stm32f413", feature = "stm32g474")))]
impl Stm32AdcChannel {
    /// Consume the unique ADC token. Hardware setup occurs in `calibrate`.
    #[must_use]
    pub const fn new(_token: crate::board::Adc) -> Self {
        Self { calibrated: false }
    }

    #[cfg(feature = "stm32g474")]
    fn wait_clear(address: usize, mask: u32) -> bool {
        for _ in 0..1_000_000 {
            // SAFETY: all callers use active-MCU ADC status/control registers.
            if unsafe { crate::mmio::read(address as *const u32) } & mask == 0 {
                return true;
            }
        }
        false
    }

    fn wait_set(address: usize, mask: u32) -> bool {
        for _ in 0..1_000_000 {
            // SAFETY: all callers use active-MCU ADC status registers.
            if unsafe { crate::mmio::read(address as *const u32) } & mask == mask {
                return true;
            }
        }
        false
    }
}

#[cfg(all(target_arch = "arm", feature = "stm32f413"))]
impl AdcChannel for Stm32AdcChannel {
    fn calibrate(&mut self) -> Result<(), AdcError> {
        const RCC_AHB1ENR: *mut u32 = 0x4002_3830 as *mut u32;
        const RCC_APB2ENR: *mut u32 = 0x4002_3844 as *mut u32;
        const GPIOA_MODER: *mut u32 = 0x4002_0000 as *mut u32;
        const ADC_CR2: *mut u32 = 0x4001_2008 as *mut u32;
        const ADC_SQR3: *mut u32 = 0x4001_2034 as *mut u32;
        // SAFETY: unique ADC token ownership and F413 register selection.
        unsafe {
            modify(RCC_AHB1ENR, 0, 1);
            modify(RCC_APB2ENR, 0, 1 << 8);
            modify(GPIOA_MODER, 0b11, 0b11);
            crate::mmio::write(ADC_SQR3, 0);
            modify(ADC_CR2, 0, 1);
        }
        self.calibrated = true;
        Ok(())
    }

    fn read_raw(&mut self) -> Result<u16, AdcError> {
        const ADC_SR: usize = 0x4001_2000;
        const ADC_CR2: *mut u32 = 0x4001_2008 as *mut u32;
        const ADC_DR: *const u32 = 0x4001_204C as *const u32;
        if !self.calibrated {
            return Err(AdcError::NotCalibrated);
        }
        // SAFETY: unique ADC token ownership.
        unsafe { modify(ADC_CR2, 0, 1 << 30) };
        if !Self::wait_set(ADC_SR, 1 << 1) {
            return Err(AdcError::HardwareTimeout);
        }
        // SAFETY: EOC is set and DR is the F413 ADC data register.
        Ok((unsafe { crate::mmio::read(ADC_DR) } & 0x0FFF) as u16)
    }
}

#[cfg(all(target_arch = "arm", feature = "stm32g474"))]
impl AdcChannel for Stm32AdcChannel {
    fn calibrate(&mut self) -> Result<(), AdcError> {
        const RCC_CR: *mut u32 = 0x4002_1000 as *mut u32;
        const RCC_CCIPR: *mut u32 = 0x4002_1088 as *mut u32;
        const RCC_AHB2ENR: *mut u32 = 0x4002_104C as *mut u32;
        const GPIOA_MODER: *mut u32 = 0x4800_0000 as *mut u32;
        const ADC_CR: *mut u32 = 0x5000_0008 as *mut u32;
        const ADC_ISR: usize = 0x5000_0000;
        const ADC_PCSEL: *mut u32 = 0x5000_001C as *mut u32;
        const ADC_SQR1: *mut u32 = 0x5000_0030 as *mut u32;
        // SAFETY: unique ADC token ownership and G474 register selection.
        unsafe {
            // Use HSI16 as the asynchronous ADC12 kernel clock. It is inside
            // the ADC's frequency limit and independent of the 170 MHz core.
            modify(RCC_CR, 0, 1 << 8);
        }
        if !Self::wait_set(RCC_CR as usize, 1 << 10) {
            return Err(AdcError::HardwareTimeout);
        }
        // SAFETY: select HSI16, enable ADC12/GPIOA, leave deep power-down,
        // then enable the ADC voltage regulator.
        unsafe {
            modify(RCC_CCIPR, 0b11 << 28, 0b11 << 28);
            modify(RCC_AHB2ENR, 0, (1 << 13) | 1);
            modify(GPIOA_MODER, 0b11, 0b11);
            modify(ADC_CR, 1 << 29, 1 << 28);
        }
        for _ in 0..5_000 {
            cortex_m::asm::nop();
        }
        // SAFETY: ADC is disabled before starting single-ended calibration.
        unsafe { modify(ADC_CR, 0, 1 << 31) };
        if !Self::wait_clear(ADC_CR as usize, 1 << 31) {
            return Err(AdcError::CalibrationFailed);
        }
        // SAFETY: channel 1 selected, then ADC enabled.
        unsafe {
            modify(ADC_PCSEL, 0, 1 << 1);
            crate::mmio::write(ADC_SQR1, 1 << 6);
            modify(ADC_CR, 0, 1);
        }
        if !Self::wait_set(ADC_ISR, 1) {
            return Err(AdcError::HardwareTimeout);
        }
        self.calibrated = true;
        Ok(())
    }

    fn read_raw(&mut self) -> Result<u16, AdcError> {
        const ADC_ISR: usize = 0x5000_0000;
        const ADC_CR: *mut u32 = 0x5000_0008 as *mut u32;
        const ADC_DR: *const u32 = 0x5000_0040 as *const u32;
        if !self.calibrated {
            return Err(AdcError::NotCalibrated);
        }
        // SAFETY: unique ADC token ownership.
        unsafe { modify(ADC_CR, 0, 1 << 2) };
        if !Self::wait_set(ADC_ISR, 1 << 2) {
            return Err(AdcError::HardwareTimeout);
        }
        // SAFETY: EOC is set and DR is the G474 ADC data register.
        Ok((unsafe { crate::mmio::read(ADC_DR) } & 0x0FFF) as u16)
    }
}

#[cfg(all(target_arch = "arm", any(feature = "stm32f413", feature = "stm32g474")))]
unsafe fn modify(register: *mut u32, clear: u32, set: u32) {
    // SAFETY: caller selects a valid active-MCU register.
    let value = unsafe { crate::mmio::read(register) };
    // SAFETY: caller has unique ownership of the peripheral.
    unsafe { crate::mmio::write(register, (value & !clear) | set) };
}

pub trait AdcChannel {
    fn calibrate(&mut self) -> Result<(), AdcError>;
    fn read_raw(&mut self) -> Result<u16, AdcError>;
}

pub struct CalibratedAdc<S> {
    source: S,
    calibrated: bool,
    maximum: u16,
}

impl<S: FnMut() -> Result<u16, AdcError>> CalibratedAdc<S> {
    pub const fn new(source: S, maximum: u16) -> Self {
        Self {
            source,
            calibrated: false,
            maximum,
        }
    }
}

impl<S: FnMut() -> Result<u16, AdcError>> AdcChannel for CalibratedAdc<S> {
    fn calibrate(&mut self) -> Result<(), AdcError> {
        let sample = (self.source)()?;
        if sample > self.maximum {
            return Err(AdcError::CalibrationFailed);
        }
        self.calibrated = true;
        Ok(())
    }

    fn read_raw(&mut self) -> Result<u16, AdcError> {
        if !self.calibrated {
            return Err(AdcError::NotCalibrated);
        }
        let sample = (self.source)()?;
        (sample <= self.maximum)
            .then_some(sample)
            .ok_or(AdcError::OutOfRange)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::Cell;
    #[test]
    fn calibration_and_twelve_bit_range_are_enforced() {
        let value = Cell::new(0u16);
        let mut adc = CalibratedAdc::new(|| Ok(value.get()), 4095);
        assert_eq!(adc.read_raw(), Err(AdcError::NotCalibrated));
        adc.calibrate().unwrap();
        value.set(4095);
        assert_eq!(adc.read_raw(), Ok(4095));
        value.set(4096);
        assert_eq!(adc.read_raw(), Err(AdcError::OutOfRange));
    }
}
