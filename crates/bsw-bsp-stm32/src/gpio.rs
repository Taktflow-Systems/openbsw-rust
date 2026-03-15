//! GPIO driver for STM32 F4/G4 — output and input pin abstraction.
//!
//! Provides a lightweight, raw-register GPIO interface for both STM32 families.
//! Pin configuration is done at init time; runtime operations are just
//! BSRR writes (set/reset) and IDR reads.

/// GPIO port base addresses.
///
/// STM32F4: GPIOA=0x40020000, stride 0x400.
/// STM32G4: GPIOA=0x48000000, stride 0x400.
#[derive(Clone, Copy)]
pub enum Port {
    A,
    B,
    C,
    D,
    E,
    F,
}

impl Port {
    /// Return the MMIO base address for this GPIO port.
    #[cfg(feature = "stm32f413")]
    const fn base(self) -> usize {
        0x4002_0000 + (self as usize) * 0x400
    }

    #[cfg(feature = "stm32g474")]
    const fn base(self) -> usize {
        0x4800_0000 + (self as usize) * 0x400
    }

    #[cfg(not(any(feature = "stm32f413", feature = "stm32g474")))]
    const fn base(self) -> usize {
        0x4002_0000 + (self as usize) * 0x400
    }

    /// RCC enable bit position for this port.
    const fn rcc_bit(self) -> u32 {
        1 << (self as u32)
    }
}

/// GPIO register offsets.
const MODER: usize = 0x00;
const OTYPER: usize = 0x04;
const OSPEEDR: usize = 0x08;
const PUPDR: usize = 0x0C;
const IDR: usize = 0x10;
const BSRR: usize = 0x18;

/// RCC base and GPIO clock enable register offset.
#[cfg(feature = "stm32f413")]
const RCC_GPIO_ENR: (usize, usize) = (0x4002_3800, 0x30); // AHB1ENR

#[cfg(feature = "stm32g474")]
const RCC_GPIO_ENR: (usize, usize) = (0x4002_1000, 0x4C); // AHB2ENR

#[cfg(not(any(feature = "stm32f413", feature = "stm32g474")))]
const RCC_GPIO_ENR: (usize, usize) = (0x4002_3800, 0x30);

#[inline(always)]
unsafe fn reg_read(addr: usize) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline(always)]
unsafe fn reg_write(addr: usize, val: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
}

#[inline(always)]
unsafe fn reg_modify(addr: usize, mask: u32, bits: u32) {
    let v = unsafe { reg_read(addr) };
    unsafe { reg_write(addr, (v & !mask) | bits) };
}

/// An output pin (push-pull, high speed).
pub struct OutputPin {
    base: usize,
    pin: u8,
}

impl OutputPin {
    /// Configure a pin as push-pull output and enable its port clock.
    pub fn new(port: Port, pin: u8) -> Self {
        assert!(pin < 16, "pin must be 0..15");
        let base = port.base();

        unsafe {
            // Enable port clock.
            let (rcc, enr_off) = RCC_GPIO_ENR;
            reg_modify(rcc + enr_off, 0, port.rcc_bit());
            let _ = reg_read(rcc + enr_off); // read-back delay

            // MODER: output (01).
            let shift = (pin as usize) * 2;
            reg_modify(base + MODER, 0b11 << shift, 0b01 << shift);

            // OTYPER: push-pull (0).
            reg_modify(base + OTYPER, 1 << pin, 0);

            // OSPEEDR: high speed (10).
            reg_modify(base + OSPEEDR, 0b11 << shift, 0b10 << shift);

            // PUPDR: no pull (00).
            reg_modify(base + PUPDR, 0b11 << shift, 0);
        }

        Self { base, pin }
    }

    /// Set the pin high.
    #[inline]
    pub fn set_high(&self) {
        unsafe { reg_write(self.base + BSRR, 1 << self.pin) };
    }

    /// Set the pin low.
    #[inline]
    pub fn set_low(&self) {
        unsafe { reg_write(self.base + BSRR, 1 << (self.pin + 16)) };
    }

    /// Toggle the pin.
    #[inline]
    pub fn toggle(&self) {
        if self.is_high() {
            self.set_low();
        } else {
            self.set_high();
        }
    }

    /// Read the current output state from ODR.
    #[inline]
    pub fn is_high(&self) -> bool {
        unsafe { (reg_read(self.base + IDR) >> self.pin) & 1 == 1 }
    }
}

/// Pull-up/pull-down configuration for input pins.
pub enum Pull {
    None,
    Up,
    Down,
}

/// An input pin.
pub struct InputPin {
    base: usize,
    pin: u8,
}

impl InputPin {
    /// Configure a pin as input with optional pull.
    pub fn new(port: Port, pin: u8, pull: Pull) -> Self {
        assert!(pin < 16, "pin must be 0..15");
        let base = port.base();

        unsafe {
            // Enable port clock.
            let (rcc, enr_off) = RCC_GPIO_ENR;
            reg_modify(rcc + enr_off, 0, port.rcc_bit());
            let _ = reg_read(rcc + enr_off);

            // MODER: input (00).
            let shift = (pin as usize) * 2;
            reg_modify(base + MODER, 0b11 << shift, 0);

            // PUPDR.
            let pull_bits = match pull {
                Pull::None => 0b00,
                Pull::Up => 0b01,
                Pull::Down => 0b10,
            };
            reg_modify(base + PUPDR, 0b11 << shift, pull_bits << shift);
        }

        Self { base, pin }
    }

    /// Read the pin level.
    #[inline]
    pub fn is_high(&self) -> bool {
        unsafe { (reg_read(self.base + IDR) >> self.pin) & 1 == 1 }
    }

    /// Read the pin level (inverted).
    #[inline]
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }
}
