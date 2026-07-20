//! EMAC CPPI descriptor representation and revision-B endian workaround.
//!
//! SPNU563A section 32.2.6 defines four 32-bit words per descriptor.
//! SPNZ232B DEVICE#54 requires software byte swapping for every CPU access to
//! CPPI RAM on TMS570 big-endian devices. The raw words stored by this type are
//! therefore always in the workaround representation.

use core::{
    mem::{align_of, size_of},
    sync::atomic::{compiler_fence, Ordering},
};

use crate::device::EMAC_CPPI_RAM;

/// Start-of-packet descriptor flag.
pub const FLAG_SOP: u32 = 0x8000_0000;
/// End-of-packet descriptor flag.
pub const FLAG_EOP: u32 = 0x4000_0000;
/// EMAC ownership flag.
pub const FLAG_OWNER: u32 = 0x2000_0000;
/// End-of-queue flag.
pub const FLAG_EOQ: u32 = 0x1000_0000;
/// Teardown-complete flag.
pub const FLAG_TEARDOWN_COMPLETE: u32 = 0x0800_0000;
/// Transmit pass-CRC flag.
pub const FLAG_PASS_CRC: u32 = 0x0400_0000;

const PACKET_LENGTH_MASK: u32 = 0x0000_ffff;

/// Descriptor preparation error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DescriptorError {
    /// Buffer address zero is invalid for an active descriptor.
    ZeroBufferAddress,
    /// Packet or buffer length is zero.
    ZeroLength,
    /// Packet or buffer length does not fit the 16-bit hardware field.
    LengthTooLarge,
    /// Descriptor capacity exceeds the 8-KiB CPPI RAM.
    RingTooLarge,
    /// A descriptor ring cannot be empty.
    EmptyRing,
}

/// Encode a CPU word for the LC4357 revision-B CPPI memory workaround.
pub const fn encode_cppi_word(cpu_word: u32) -> u32 {
    cpu_word.swap_bytes()
}

/// Decode a raw CPPI word read by the CPU.
pub const fn decode_cppi_word(raw_word: u32) -> u32 {
    raw_word.swap_bytes()
}

/// One 16-byte EMAC packet descriptor.
#[repr(C, align(16))]
#[derive(Clone, Copy)]
pub struct CppiDescriptor {
    raw_words: [u32; 4],
}

impl CppiDescriptor {
    /// Cleared descriptor not owned by either queue.
    pub const EMPTY: Self = Self { raw_words: [0; 4] };

    fn read_raw(&self, index: usize) -> u32 {
        let pointer = core::ptr::addr_of!(self.raw_words[index]);
        // SAFETY: `pointer` refers to one aligned word inside a live
        // descriptor. Volatile access is required because the EMAC can update
        // CPPI RAM independently of the CPU.
        unsafe { core::ptr::read_volatile(pointer) }
    }

    fn write_raw(&mut self, index: usize, value: u32) {
        let pointer = core::ptr::addr_of_mut!(self.raw_words[index]);
        // SAFETY: `pointer` refers to one aligned, exclusively borrowed word
        // inside a live descriptor. Volatile access prevents elision or
        // reordering across the device-visible write.
        unsafe { core::ptr::write_volatile(pointer, value) }
    }

    fn read_cpu(&self, index: usize) -> u32 {
        decode_cppi_word(self.read_raw(index))
    }

    fn write_cpu(&mut self, index: usize, value: u32) {
        self.write_raw(index, encode_cppi_word(value));
    }

    /// Raw workaround word for map/debug checks.
    pub fn raw_word(&self, index: usize) -> u32 {
        self.read_raw(index)
    }

    /// Clear every descriptor word.
    pub fn clear(&mut self) {
        for index in 0..self.raw_words.len() {
            self.write_raw(index, 0);
        }
    }

    /// Prepare a single-buffer transmit packet, still owned by software.
    pub fn prepare_single_tx(
        &mut self,
        buffer_address: u32,
        length: usize,
        pass_crc: bool,
    ) -> Result<(), DescriptorError> {
        let length = checked_length(buffer_address, length)?;
        self.write_cpu(0, 0);
        self.write_cpu(1, buffer_address);
        self.write_cpu(2, u32::from(length));
        let crc_flag = if pass_crc { FLAG_PASS_CRC } else { 0 };
        self.write_cpu(3, FLAG_SOP | FLAG_EOP | crc_flag | u32::from(length));
        Ok(())
    }

    /// Prepare one receive buffer, still owned by software.
    pub fn prepare_rx(
        &mut self,
        next_descriptor: u32,
        buffer_address: u32,
        capacity: usize,
    ) -> Result<(), DescriptorError> {
        let capacity = checked_length(buffer_address, capacity)?;
        self.write_cpu(0, next_descriptor);
        self.write_cpu(1, buffer_address);
        self.write_cpu(2, u32::from(capacity));
        self.write_cpu(3, 0);
        Ok(())
    }

    /// Next descriptor address decoded for the CPU.
    pub fn next_descriptor(&self) -> u32 {
        self.read_cpu(0)
    }

    /// Packet buffer address decoded for the CPU.
    pub fn buffer_address(&self) -> u32 {
        self.read_cpu(1)
    }

    /// Buffer capacity/length decoded for the CPU.
    pub fn buffer_length(&self) -> u16 {
        self.read_cpu(2) as u16
    }

    /// Packet flags decoded for the CPU.
    pub fn flags(&self) -> u32 {
        self.read_cpu(3) & !PACKET_LENGTH_MASK
    }

    /// Packet length decoded for the CPU.
    pub fn packet_length(&self) -> u16 {
        self.read_cpu(3) as u16
    }

    /// Whether the descriptor packet is currently owned by the EMAC.
    pub fn is_owned_by_emac(&self) -> bool {
        self.read_cpu(3) & FLAG_OWNER != 0
    }

    /// Publish all prepared fields and transfer the packet to the EMAC.
    pub fn release_to_emac(&mut self) {
        dma_release_barrier();
        let flags_and_length = self.read_cpu(3) | FLAG_OWNER;
        self.write_cpu(3, flags_and_length);
        dma_release_barrier();
    }

    /// Observe completion and acquire DMA-written fields when ownership clears.
    pub fn try_reclaim(&self) -> bool {
        if self.is_owned_by_emac() {
            return false;
        }
        dma_acquire_barrier();
        true
    }
}

fn checked_length(buffer_address: u32, length: usize) -> Result<u16, DescriptorError> {
    if buffer_address == 0 {
        return Err(DescriptorError::ZeroBufferAddress);
    }
    if length == 0 {
        return Err(DescriptorError::ZeroLength);
    }
    u16::try_from(length).map_err(|_| DescriptorError::LengthTooLarge)
}

fn dma_release_barrier() {
    compiler_fence(Ordering::Release);
    arm_data_memory_barrier();
}

fn dma_acquire_barrier() {
    arm_data_memory_barrier();
    compiler_fence(Ordering::Acquire);
}

#[inline]
fn arm_data_memory_barrier() {
    #[cfg(target_arch = "arm")]
    // SAFETY: `dmb sy` has no operands, does not touch the stack, and is used
    // only to order CPU memory transactions against the EMAC DMA master.
    unsafe {
        core::arch::asm!("dmb sy", options(nostack, preserves_flags));
    }
}

/// Statically allocated CPPI descriptor ring.
#[repr(C, align(16))]
pub struct CppiRing<const DESCRIPTORS: usize> {
    descriptors: [CppiDescriptor; DESCRIPTORS],
}

impl<const DESCRIPTORS: usize> CppiRing<DESCRIPTORS> {
    /// Construct a cleared ring that fits entirely in the LC4357 CPPI RAM.
    pub fn try_new() -> Result<Self, DescriptorError> {
        if DESCRIPTORS == 0 {
            return Err(DescriptorError::EmptyRing);
        }
        if DESCRIPTORS > EMAC_CPPI_RAM.len() as usize / size_of::<CppiDescriptor>() {
            return Err(DescriptorError::RingTooLarge);
        }
        Ok(Self {
            descriptors: [CppiDescriptor::EMPTY; DESCRIPTORS],
        })
    }

    /// Number of descriptors.
    pub const fn len(&self) -> usize {
        DESCRIPTORS
    }

    /// Rings constructed by [`Self::try_new`] are never empty.
    pub const fn is_empty(&self) -> bool {
        DESCRIPTORS == 0
    }

    /// Immutable descriptor access.
    pub fn get(&self, index: usize) -> Option<&CppiDescriptor> {
        self.descriptors.get(index)
    }

    /// Mutable descriptor access while it remains software-owned.
    pub fn get_mut(&mut self, index: usize) -> Option<&mut CppiDescriptor> {
        self.descriptors.get_mut(index)
    }
}

const _: () = {
    assert!(size_of::<CppiDescriptor>() == 16);
    assert!(align_of::<CppiDescriptor>() == 16);
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn descriptor_layout_and_device_54_codec_are_exact() {
        assert_eq!(size_of::<CppiDescriptor>(), 16);
        assert_eq!(align_of::<CppiDescriptor>(), 16);
        assert_eq!(encode_cppi_word(0x1234_5678), 0x7856_3412);
        assert_eq!(decode_cppi_word(0x7856_3412), 0x1234_5678);
    }

    #[test]
    fn transmit_ownership_is_published_last() {
        let mut descriptor = CppiDescriptor::EMPTY;
        descriptor
            .prepare_single_tx(0x0800_1000, 60, false)
            .unwrap();
        assert_eq!(descriptor.buffer_address(), 0x0800_1000);
        assert_eq!(descriptor.buffer_length(), 60);
        assert_eq!(descriptor.packet_length(), 60);
        assert_eq!(descriptor.flags(), FLAG_SOP | FLAG_EOP);
        assert!(!descriptor.is_owned_by_emac());
        descriptor.release_to_emac();
        assert!(descriptor.is_owned_by_emac());
        assert_eq!(descriptor.raw_word(1), encode_cppi_word(0x0800_1000));
    }

    #[test]
    fn invalid_buffers_and_lengths_fail_closed() {
        let mut descriptor = CppiDescriptor::EMPTY;
        assert_eq!(
            descriptor.prepare_single_tx(0, 60, false),
            Err(DescriptorError::ZeroBufferAddress)
        );
        assert_eq!(
            descriptor.prepare_single_tx(0x0800_1000, 0, false),
            Err(DescriptorError::ZeroLength)
        );
        assert_eq!(
            descriptor.prepare_single_tx(0x0800_1000, 65_536, false),
            Err(DescriptorError::LengthTooLarge)
        );
    }

    #[test]
    fn ring_capacity_is_bounded_by_cppi_ram() {
        assert!(CppiRing::<1>::try_new().is_ok());
        assert!(CppiRing::<512>::try_new().is_ok());
        assert!(matches!(
            CppiRing::<0>::try_new(),
            Err(DescriptorError::EmptyRing)
        ));
        assert!(matches!(
            CppiRing::<513>::try_new(),
            Err(DescriptorError::RingTooLarge)
        ));
    }
}
