//! Audited volatile-access boundary for STM32 driver modules (G02).
//!
//! Address selection and peripheral ownership remain the responsibility of
//! each typed driver. These are the only helpers that perform volatile CPU
//! loads and stores for memory-mapped registers.

#[inline(always)]
pub(crate) unsafe fn read<T: Copy>(source: *const T) -> T {
    // SAFETY: the driver caller proves alignment, validity, and read access.
    unsafe { core::ptr::read_volatile(source) }
}

#[inline(always)]
pub(crate) unsafe fn write<T>(destination: *mut T, value: T) {
    // SAFETY: the driver caller proves alignment, validity, and write access.
    unsafe { core::ptr::write_volatile(destination, value) };
}
