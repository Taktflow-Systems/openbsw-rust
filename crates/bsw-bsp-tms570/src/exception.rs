//! Integrity-protected retained exception handoff for Cortex-R5F.
//!
//! The target exception entries are terminal: they capture the banked entry
//! state, publish this record with a two-phase commit, execute a full-system
//! barrier, and request a software reset. They never expose MMIO or retained
//! storage pointers to application code. The pure record logic in this module
//! is also used by host tests to reject stale, corrupt, recursive, unknown, or
//! architecturally inconsistent handoffs before an application can act on
//! them.

use core::mem::{align_of, size_of};

const MAGIC: u32 = 0x544d_5346;
const FORMAT_VERSION: u32 = 1;
const RECORD_WORDS: usize = 40;
const FORMAT: u32 = (FORMAT_VERSION << 16) | RECORD_WORDS as u32;
const CRC_WORDS: usize = 36;

const STATE_ARMED: u32 = 0x4152_4d44;
const STATE_CAPTURING: u32 = 0x4341_5054;
const STATE_COMPLETE: u32 = 0x434f_4d50;
const STATE_POISONED: u32 = 0x504f_4953;

const COMMIT_ARMED: u32 = 0x4152_4d43;
const COMMIT_COMPLETE: u32 = 0x434f_4d54;
const COMMIT_POISONED: u32 = 0x504f_434d;

const INDEX_MAGIC: usize = 0;
const INDEX_FORMAT: usize = 1;
const INDEX_STATE: usize = 2;
const INDEX_STATE_INVERSE: usize = 3;
const INDEX_GENERATION: usize = 4;
const INDEX_GENERATION_INVERSE: usize = 5;
const INDEX_CLASS: usize = 6;
const INDEX_CLASS_INVERSE: usize = 7;
const INDEX_ENTRY_CPSR: usize = 8;
const INDEX_SOURCE_SPSR: usize = 9;
const INDEX_EXCEPTION_LR: usize = 10;
const INDEX_EXCEPTION_PC: usize = 11;
const INDEX_RESUME_PC: usize = 12;
const INDEX_EXCEPTION_SP: usize = 13;
const INDEX_SOURCE_SP: usize = 14;
const INDEX_SOURCE_LR: usize = 15;
const INDEX_REGISTERS: usize = 16;
const INDEX_DFSR: usize = 29;
const INDEX_DFAR: usize = 30;
const INDEX_IFSR: usize = 31;
const INDEX_IFAR: usize = 32;
const INDEX_ADFSR: usize = 33;
const INDEX_AIFSR: usize = 34;
const INDEX_FLAGS: usize = 35;
const INDEX_CRC: usize = 36;
const INDEX_CRC_INVERSE: usize = 37;
const INDEX_COMMIT: usize = 38;
const INDEX_COMMIT_INVERSE: usize = 39;

/// The source System/User banked SP and LR fields are valid.
pub const FLAG_SOURCE_BANK_VALID: u32 = 1 << 0;
/// DFAR contains a synchronous data-fault address.
pub const FLAG_DFAR_VALID: u32 = 1 << 1;
/// IFAR contains a synchronous instruction-fault address.
pub const FLAG_IFAR_VALID: u32 = 1 << 2;
/// A debugger caught the fixed vector and redirected PC to the RAM handler.
///
/// This flag is reserved for the bounded RAM-loaded physical role. A
/// production direct-vector capture never sets it.
pub const FLAG_DEBUG_VECTOR_REDIRECT: u32 = 1 << 3;

const FLAG_POISON_CORRUPT: u32 = 1 << 16;
const FLAG_POISON_RECURSIVE: u32 = 1 << 17;
const FLAG_POISON_UNKNOWN: u32 = 1 << 18;

const CPSR_MODE_MASK: u32 = 0x1f;
const CPSR_T: u32 = 1 << 5;
const CPSR_E: u32 = 1 << 9;
const CPSR_J: u32 = 1 << 24;
const MODE_USER: u32 = 0x10;
const MODE_FIQ: u32 = 0x11;
const MODE_IRQ: u32 = 0x12;
const MODE_SUPERVISOR: u32 = 0x13;
const MODE_ABORT: u32 = 0x17;
const MODE_UNDEFINED: u32 = 0x1b;
const MODE_SYSTEM: u32 = 0x1f;

/// Exact retained-record size consumed by target assembly and linkers.
pub const RETAINED_RECORD_BYTES: usize = RECORD_WORDS * size_of::<u32>();

/// Exception vector captured by the terminal handoff.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum ExceptionClass {
    /// Undefined A32 instruction, entered in Undefined mode.
    Undefined = 1,
    /// A32 supervisor call, entered in Supervisor mode.
    SupervisorCall = 2,
    /// Synchronous instruction-fetch abort, entered in Abort mode.
    PrefetchAbort = 3,
    /// Data abort, entered in Abort mode.
    DataAbort = 4,
}

impl ExceptionClass {
    const fn from_raw(raw: u32) -> Option<Self> {
        match raw {
            1 => Some(Self::Undefined),
            2 => Some(Self::SupervisorCall),
            3 => Some(Self::PrefetchAbort),
            4 => Some(Self::DataAbort),
            _ => None,
        }
    }

    const fn entry_mode(self) -> u32 {
        match self {
            Self::Undefined => MODE_UNDEFINED,
            Self::SupervisorCall => MODE_SUPERVISOR,
            Self::PrefetchAbort | Self::DataAbort => MODE_ABORT,
        }
    }

    const fn corrected_pcs(self, exception_lr: u32) -> (u32, u32) {
        match self {
            Self::Undefined => {
                let instruction = exception_lr.wrapping_sub(4);
                (instruction, instruction)
            }
            Self::SupervisorCall => (exception_lr.wrapping_sub(4), exception_lr),
            Self::PrefetchAbort => {
                let instruction = exception_lr.wrapping_sub(4);
                (instruction, instruction)
            }
            Self::DataAbort => {
                let instruction = exception_lr.wrapping_sub(8);
                (instruction, instruction)
            }
        }
    }
}

/// Captured CP15 fault registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaultRegisters {
    /// Data Fault Status Register.
    pub dfsr: u32,
    /// Data Fault Address Register; valid only when `FLAG_DFAR_VALID` is set.
    pub dfar: u32,
    /// Instruction Fault Status Register.
    pub ifsr: u32,
    /// Instruction Fault Address Register; valid only when `FLAG_IFAR_VALID` is set.
    pub ifar: u32,
    /// Implementation-defined Auxiliary Data Fault Status Register.
    pub adfsr: u32,
    /// Implementation-defined Auxiliary Instruction Fault Status Register.
    pub aifsr: u32,
}

/// Register snapshot presented to the host-testable capture state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CaptureSnapshot {
    /// CPSR after entering the banked exception mode and masking exceptions.
    pub entry_cpsr: u32,
    /// SPSR of the banked exception mode, containing the source CPSR.
    pub source_spsr: u32,
    /// Banked exception LR before any call instruction can replace it.
    pub exception_lr: u32,
    /// Banked exception SP before the assembly entry frame was allocated.
    pub exception_sp: u32,
    /// User/System banked SP when the source mode makes it meaningful.
    pub source_sp: u32,
    /// User/System banked LR when the source mode makes it meaningful.
    pub source_lr: u32,
    /// Source r0-r12, saved before the handler clobbers a general register.
    pub registers: [u32; 13],
    /// CP15 abort diagnostics.
    pub faults: FaultRegisters,
    /// Capture metadata flags. Poison bits are reserved to the state machine.
    pub flags: u32,
}

/// A validated retained exception handed to startup/application policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RetainedException {
    /// Monotonic application-provided generation used to reject stale data.
    pub generation: u32,
    /// Classified vector.
    pub class: ExceptionClass,
    /// CPSR in the banked handler.
    pub entry_cpsr: u32,
    /// CPSR immediately before exception entry.
    pub source_spsr: u32,
    /// Raw banked LR supplied by the architecture.
    pub exception_lr: u32,
    /// Address of the faulting A32 instruction or SVC instruction.
    pub exception_pc: u32,
    /// Architecturally preferred resume address for this exception class.
    pub resume_pc: u32,
    /// Banked exception SP before the entry frame.
    pub exception_sp: u32,
    /// Source User/System SP, meaningful only with `FLAG_SOURCE_BANK_VALID`.
    pub source_sp: u32,
    /// Source User/System LR, meaningful only with `FLAG_SOURCE_BANK_VALID`.
    pub source_lr: u32,
    /// Source r0-r12.
    pub registers: [u32; 13],
    /// CP15 fault diagnostics.
    pub faults: FaultRegisters,
    /// Validity and physical-probe metadata.
    pub flags: u32,
}

/// Fail-closed retained-handoff rejection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RetainedError {
    /// No armed or completed record exists.
    Empty,
    /// Capture was armed but no exception committed a handoff.
    Armed,
    /// Capture was interrupted while already writing the record.
    Recursive,
    /// Magic, inverse fields, CRC, commit, or context invariants failed.
    Corrupt,
    /// The record belongs to a different application generation.
    Stale,
    /// The vector/class field is not one of the four accepted classes.
    UnknownClass,
    /// The source or handler state is not A32 big-endian ARMv7-R state.
    UnsupportedState,
}

/// Exact target-visible retained image.
///
/// Fields remain private so callers cannot manufacture a trusted handoff.
/// Target assembly uses the fixed word offsets asserted below and by the
/// disassembly checker.
#[derive(Clone, Copy)]
#[repr(C, align(32))]
pub struct RetainedExceptionRecord {
    words: [u32; RECORD_WORDS],
}

impl RetainedExceptionRecord {
    /// All-zero state produced by L2 SRAM/ECC auto-initialization on cold boot.
    pub const EMPTY: Self = Self {
        words: [0; RECORD_WORDS],
    };

    /// Prepare a fresh generation for exactly one terminal exception capture.
    pub fn arm(&mut self, generation: u32) {
        self.words = [0; RECORD_WORDS];
        self.words[INDEX_MAGIC] = MAGIC;
        self.words[INDEX_FORMAT] = FORMAT;
        self.words[INDEX_STATE] = STATE_ARMED;
        self.words[INDEX_STATE_INVERSE] = !STATE_ARMED;
        self.words[INDEX_GENERATION] = generation;
        self.words[INDEX_GENERATION_INVERSE] = !generation;
        self.words[INDEX_CLASS_INVERSE] = u32::MAX;
        self.seal(COMMIT_ARMED);
    }

    /// Model the assembly handler's bounded state transition on the host.
    pub fn capture(&mut self, class: ExceptionClass, snapshot: &CaptureSnapshot) {
        self.capture_raw(class as u32, snapshot);
    }

    /// Validate and classify a post-reset record for one expected generation.
    pub fn classify(&self, expected_generation: u32) -> Result<RetainedException, RetainedError> {
        if self.words.iter().all(|word| *word == 0) {
            return Err(RetainedError::Empty);
        }
        self.validate_envelope()?;
        if self.words[INDEX_GENERATION] != expected_generation {
            return Err(RetainedError::Stale);
        }
        match self.words[INDEX_STATE] {
            STATE_ARMED => Err(RetainedError::Armed),
            STATE_CAPTURING => Err(RetainedError::Recursive),
            STATE_POISONED => self.poison_error(),
            STATE_COMPLETE => self.classify_complete(),
            _ => Err(RetainedError::Corrupt),
        }
    }

    fn capture_raw(&mut self, raw_class: u32, snapshot: &CaptureSnapshot) {
        match self.validate_armed() {
            Ok(()) => {}
            Err(RetainedError::Recursive) => {
                self.poison(FLAG_POISON_RECURSIVE);
                return;
            }
            Err(_) => {
                self.poison(FLAG_POISON_CORRUPT);
                return;
            }
        }
        let Some(class) = ExceptionClass::from_raw(raw_class) else {
            self.poison(FLAG_POISON_UNKNOWN);
            return;
        };

        self.words[INDEX_STATE] = STATE_CAPTURING;
        self.words[INDEX_STATE_INVERSE] = !STATE_CAPTURING;
        self.words[INDEX_COMMIT] = 0;
        self.words[INDEX_COMMIT_INVERSE] = 0;

        self.words[INDEX_CLASS] = raw_class;
        self.words[INDEX_CLASS_INVERSE] = !raw_class;
        self.words[INDEX_ENTRY_CPSR] = snapshot.entry_cpsr;
        self.words[INDEX_SOURCE_SPSR] = snapshot.source_spsr;
        self.words[INDEX_EXCEPTION_LR] = snapshot.exception_lr;
        let (exception_pc, resume_pc) = class.corrected_pcs(snapshot.exception_lr);
        self.words[INDEX_EXCEPTION_PC] = exception_pc;
        self.words[INDEX_RESUME_PC] = resume_pc;
        self.words[INDEX_EXCEPTION_SP] = snapshot.exception_sp;
        self.words[INDEX_SOURCE_SP] = snapshot.source_sp;
        self.words[INDEX_SOURCE_LR] = snapshot.source_lr;
        self.words[INDEX_REGISTERS..INDEX_REGISTERS + 13].copy_from_slice(&snapshot.registers);
        self.words[INDEX_DFSR] = snapshot.faults.dfsr;
        self.words[INDEX_DFAR] = snapshot.faults.dfar;
        self.words[INDEX_IFSR] = snapshot.faults.ifsr;
        self.words[INDEX_IFAR] = snapshot.faults.ifar;
        self.words[INDEX_ADFSR] = snapshot.faults.adfsr;
        self.words[INDEX_AIFSR] = snapshot.faults.aifsr;
        self.words[INDEX_FLAGS] = normalized_flags(class, snapshot);
        self.words[INDEX_STATE] = STATE_COMPLETE;
        self.words[INDEX_STATE_INVERSE] = !STATE_COMPLETE;
        self.seal(COMMIT_COMPLETE);
    }

    fn validate_armed(&self) -> Result<(), RetainedError> {
        if self.words[INDEX_STATE] == STATE_CAPTURING {
            return Err(RetainedError::Recursive);
        }
        self.validate_envelope()?;
        if self.words[INDEX_STATE] != STATE_ARMED || self.words[INDEX_COMMIT] != COMMIT_ARMED {
            return Err(RetainedError::Corrupt);
        }
        Ok(())
    }

    fn validate_envelope(&self) -> Result<(), RetainedError> {
        if self.words[INDEX_MAGIC] != MAGIC || self.words[INDEX_FORMAT] != FORMAT {
            return Err(RetainedError::Corrupt);
        }
        for (value, inverse) in [
            (INDEX_STATE, INDEX_STATE_INVERSE),
            (INDEX_GENERATION, INDEX_GENERATION_INVERSE),
            (INDEX_CLASS, INDEX_CLASS_INVERSE),
            (INDEX_CRC, INDEX_CRC_INVERSE),
            (INDEX_COMMIT, INDEX_COMMIT_INVERSE),
        ] {
            if self.words[value] != !self.words[inverse] {
                return Err(RetainedError::Corrupt);
            }
        }
        let expected_commit = match self.words[INDEX_STATE] {
            STATE_ARMED => COMMIT_ARMED,
            STATE_COMPLETE => COMMIT_COMPLETE,
            STATE_POISONED => COMMIT_POISONED,
            STATE_CAPTURING => return Err(RetainedError::Recursive),
            _ => return Err(RetainedError::Corrupt),
        };
        if self.words[INDEX_COMMIT] != expected_commit
            || crc32_be_words(&self.words[..CRC_WORDS]) != self.words[INDEX_CRC]
        {
            return Err(RetainedError::Corrupt);
        }
        Ok(())
    }

    fn classify_complete(&self) -> Result<RetainedException, RetainedError> {
        let Some(class) = ExceptionClass::from_raw(self.words[INDEX_CLASS]) else {
            return Err(RetainedError::UnknownClass);
        };
        validate_architecture(class, &self.words)?;
        let mut registers = [0; 13];
        registers.copy_from_slice(&self.words[INDEX_REGISTERS..INDEX_REGISTERS + 13]);
        Ok(RetainedException {
            generation: self.words[INDEX_GENERATION],
            class,
            entry_cpsr: self.words[INDEX_ENTRY_CPSR],
            source_spsr: self.words[INDEX_SOURCE_SPSR],
            exception_lr: self.words[INDEX_EXCEPTION_LR],
            exception_pc: self.words[INDEX_EXCEPTION_PC],
            resume_pc: self.words[INDEX_RESUME_PC],
            exception_sp: self.words[INDEX_EXCEPTION_SP],
            source_sp: self.words[INDEX_SOURCE_SP],
            source_lr: self.words[INDEX_SOURCE_LR],
            registers,
            faults: FaultRegisters {
                dfsr: self.words[INDEX_DFSR],
                dfar: self.words[INDEX_DFAR],
                ifsr: self.words[INDEX_IFSR],
                ifar: self.words[INDEX_IFAR],
                adfsr: self.words[INDEX_ADFSR],
                aifsr: self.words[INDEX_AIFSR],
            },
            flags: self.words[INDEX_FLAGS],
        })
    }

    fn poison(&mut self, reason: u32) {
        let generation = if self.words[INDEX_GENERATION] == !self.words[INDEX_GENERATION_INVERSE] {
            self.words[INDEX_GENERATION]
        } else {
            0
        };
        self.words = [0; RECORD_WORDS];
        self.words[INDEX_MAGIC] = MAGIC;
        self.words[INDEX_FORMAT] = FORMAT;
        self.words[INDEX_STATE] = STATE_POISONED;
        self.words[INDEX_STATE_INVERSE] = !STATE_POISONED;
        self.words[INDEX_GENERATION] = generation;
        self.words[INDEX_GENERATION_INVERSE] = !generation;
        self.words[INDEX_CLASS_INVERSE] = u32::MAX;
        self.words[INDEX_FLAGS] = reason;
        self.seal(COMMIT_POISONED);
    }

    fn poison_error(&self) -> Result<RetainedException, RetainedError> {
        let flags = self.words[INDEX_FLAGS];
        if flags & FLAG_POISON_RECURSIVE != 0 {
            Err(RetainedError::Recursive)
        } else if flags & FLAG_POISON_UNKNOWN != 0 {
            Err(RetainedError::UnknownClass)
        } else {
            Err(RetainedError::Corrupt)
        }
    }

    fn seal(&mut self, commit: u32) {
        let crc = crc32_be_words(&self.words[..CRC_WORDS]);
        self.words[INDEX_CRC] = crc;
        self.words[INDEX_CRC_INVERSE] = !crc;
        self.words[INDEX_COMMIT] = commit;
        self.words[INDEX_COMMIT_INVERSE] = !commit;
    }
}

fn normalized_flags(class: ExceptionClass, snapshot: &CaptureSnapshot) -> u32 {
    let mut flags = snapshot.flags & FLAG_DEBUG_VECTOR_REDIRECT;
    let source_mode = snapshot.source_spsr & CPSR_MODE_MASK;
    if matches!(source_mode, MODE_USER | MODE_SYSTEM) {
        flags |= FLAG_SOURCE_BANK_VALID;
    }
    match class {
        ExceptionClass::PrefetchAbort => flags |= FLAG_IFAR_VALID,
        ExceptionClass::DataAbort if data_fault_address_valid(snapshot.faults.dfsr) => {
            flags |= FLAG_DFAR_VALID;
        }
        _ => {}
    }
    flags
}

const fn data_fault_address_valid(dfsr: u32) -> bool {
    let status = (dfsr & 0x0f) | ((dfsr >> 6) & 0x10);
    status != 0x16 && status != 0x18
}

fn validate_architecture(
    class: ExceptionClass,
    words: &[u32; RECORD_WORDS],
) -> Result<(), RetainedError> {
    let entry = words[INDEX_ENTRY_CPSR];
    let source = words[INDEX_SOURCE_SPSR];
    if entry & CPSR_MODE_MASK != class.entry_mode()
        || entry & (CPSR_T | CPSR_J) != 0
        || entry & CPSR_E == 0
        || source & (CPSR_T | CPSR_J) != 0
        || source & CPSR_E == 0
        || !valid_source_mode(source & CPSR_MODE_MASK)
    {
        return Err(RetainedError::UnsupportedState);
    }
    let (exception_pc, resume_pc) = class.corrected_pcs(words[INDEX_EXCEPTION_LR]);
    if words[INDEX_EXCEPTION_PC] != exception_pc || words[INDEX_RESUME_PC] != resume_pc {
        return Err(RetainedError::Corrupt);
    }
    let flags = words[INDEX_FLAGS];
    let allowed_flags =
        FLAG_SOURCE_BANK_VALID | FLAG_DFAR_VALID | FLAG_IFAR_VALID | FLAG_DEBUG_VECTOR_REDIRECT;
    let source_bank_expected = matches!(source & CPSR_MODE_MASK, MODE_USER | MODE_SYSTEM);
    if flags & !allowed_flags != 0
        || source_bank_expected != (flags & FLAG_SOURCE_BANK_VALID != 0)
        || (!source_bank_expected && (words[INDEX_SOURCE_SP] != 0 || words[INDEX_SOURCE_LR] != 0))
        || (class == ExceptionClass::PrefetchAbort) != (flags & FLAG_IFAR_VALID != 0)
        || (class != ExceptionClass::DataAbort && flags & FLAG_DFAR_VALID != 0)
    {
        return Err(RetainedError::Corrupt);
    }
    if class == ExceptionClass::DataAbort
        && data_fault_address_valid(words[INDEX_DFSR]) != (flags & FLAG_DFAR_VALID != 0)
    {
        return Err(RetainedError::Corrupt);
    }
    Ok(())
}

const fn valid_source_mode(mode: u32) -> bool {
    matches!(
        mode,
        MODE_USER
            | MODE_FIQ
            | MODE_IRQ
            | MODE_SUPERVISOR
            | MODE_ABORT
            | MODE_UNDEFINED
            | MODE_SYSTEM
    )
}

/// Non-reflected CRC-32 over the target's BE32 byte representation.
///
/// Polynomial `0x04c11db7`, initial value and final XOR `0xffff_ffff`.
fn crc32_be_words(words: &[u32]) -> u32 {
    let mut crc = u32::MAX;
    for word in words {
        for byte in word.to_be_bytes() {
            crc ^= u32::from(byte) << 24;
            for _ in 0..8 {
                crc = if crc & 0x8000_0000 != 0 {
                    (crc << 1) ^ 0x04c1_1db7
                } else {
                    crc << 1
                };
            }
        }
    }
    !crc
}

#[cfg(target_arch = "arm")]
#[used]
#[link_section = ".retained.tms570_exception"]
#[no_mangle]
static mut __tms570_exception_record: RetainedExceptionRecord = RetainedExceptionRecord::EMPTY;

/// Arm the private target record without exposing its address.
#[cfg(target_arch = "arm")]
pub fn arm_retained_exception(generation: u32) {
    let mut armed = RetainedExceptionRecord::EMPTY;
    armed.arm(generation);
    let source = &armed as *const RetainedExceptionRecord as *const u32;
    let destination = &raw mut __tms570_exception_record as *mut u32;
    for index in 0..RECORD_WORDS {
        // SAFETY: both pointers refer to aligned 40-word record storage. The
        // destination is owned only by startup while exceptions are masked.
        unsafe { core::ptr::write_volatile(destination.add(index), source.add(index).read()) };
    }
    // SAFETY: publication must reach ECC-protected SRAM before the commit can
    // arm an asynchronous exception entry.
    unsafe { core::arch::asm!("dsb sy", "isb", options(nostack, preserves_flags)) };
}

/// Read and validate the private target record after reset.
#[cfg(target_arch = "arm")]
pub fn retained_exception(expected_generation: u32) -> Result<RetainedException, RetainedError> {
    let source = &raw const __tms570_exception_record as *const u32;
    let mut snapshot = RetainedExceptionRecord::EMPTY;
    let destination = &raw mut snapshot.words as *mut u32;
    for index in 0..RECORD_WORDS {
        // SAFETY: the private retained symbol and local record are aligned and
        // exactly 40 words. Volatile reads prevent reset evidence elision.
        unsafe {
            destination
                .add(index)
                .write(core::ptr::read_volatile(source.add(index)))
        };
    }
    snapshot.classify(expected_generation)
}

const _: () = {
    assert!(RETAINED_RECORD_BYTES == 160);
    assert!(size_of::<RetainedExceptionRecord>() == RETAINED_RECORD_BYTES);
    assert!(align_of::<RetainedExceptionRecord>() == 32);
    assert!(INDEX_FLAGS + 1 == CRC_WORDS);
    assert!(INDEX_COMMIT_INVERSE + 1 == RECORD_WORDS);
};

#[cfg(test)]
mod tests {
    use super::*;

    const GENERATION: u32 = 0x1020_3040;
    const BIG_ENDIAN_SYSTEM: u32 = CPSR_E | MODE_SYSTEM;

    fn snapshot(class: ExceptionClass) -> CaptureSnapshot {
        CaptureSnapshot {
            entry_cpsr: CPSR_E | class.entry_mode(),
            source_spsr: BIG_ENDIAN_SYSTEM,
            exception_lr: 0x0800_1040,
            exception_sp: 0x0807_f000,
            source_sp: 0x0807_e000,
            source_lr: 0x0800_2000,
            registers: core::array::from_fn(|index| 0x1000_0000 + index as u32),
            faults: FaultRegisters {
                dfsr: 0x0000_0008,
                dfar: 0x0400_0000,
                ifsr: 0x0000_0008,
                ifar: 0x0400_0000,
                adfsr: 0xaaaa_0001,
                aifsr: 0xbbbb_0002,
            },
            flags: 0,
        }
    }

    #[test]
    fn every_a32_lr_correction_and_banked_mode_is_exact() {
        for class in [
            ExceptionClass::Undefined,
            ExceptionClass::SupervisorCall,
            ExceptionClass::PrefetchAbort,
            ExceptionClass::DataAbort,
        ] {
            let mut record = RetainedExceptionRecord::EMPTY;
            record.arm(GENERATION);
            record.capture(class, &snapshot(class));
            let retained = record.classify(GENERATION).unwrap();
            let expected_fault = snapshot(class).exception_lr
                - if class == ExceptionClass::DataAbort {
                    8
                } else {
                    4
                };
            assert_eq!(retained.class, class);
            assert_eq!(retained.exception_pc, expected_fault);
            assert_eq!(
                retained.resume_pc,
                if class == ExceptionClass::SupervisorCall {
                    snapshot(class).exception_lr
                } else {
                    expected_fault
                }
            );
            assert_eq!(retained.registers, snapshot(class).registers);
            assert_ne!(retained.flags & FLAG_SOURCE_BANK_VALID, 0);
        }
    }

    #[test]
    fn instruction_and_data_fault_address_validity_is_classified() {
        let mut prefetch = RetainedExceptionRecord::EMPTY;
        prefetch.arm(GENERATION);
        prefetch.capture(
            ExceptionClass::PrefetchAbort,
            &snapshot(ExceptionClass::PrefetchAbort),
        );
        assert_ne!(
            prefetch.classify(GENERATION).unwrap().flags & FLAG_IFAR_VALID,
            0
        );

        let mut asynchronous = snapshot(ExceptionClass::DataAbort);
        asynchronous.faults.dfsr = 0x0000_0406;
        let mut data = RetainedExceptionRecord::EMPTY;
        data.arm(GENERATION);
        data.capture(ExceptionClass::DataAbort, &asynchronous);
        assert_eq!(
            data.classify(GENERATION).unwrap().flags & FLAG_DFAR_VALID,
            0
        );
    }

    #[test]
    fn corrupt_stale_recursive_and_unknown_records_fail_closed() {
        let mut record = RetainedExceptionRecord::EMPTY;
        record.arm(GENERATION);
        assert_eq!(record.classify(GENERATION), Err(RetainedError::Armed));
        assert_eq!(
            record.classify(GENERATION.wrapping_add(1)),
            Err(RetainedError::Stale)
        );

        let mut corrupt = record;
        corrupt.words[INDEX_SOURCE_SP] ^= 1;
        assert_eq!(corrupt.classify(GENERATION), Err(RetainedError::Corrupt));

        let mut recursive = record;
        recursive.words[INDEX_STATE] = STATE_CAPTURING;
        recursive.words[INDEX_STATE_INVERSE] = !STATE_CAPTURING;
        recursive.capture(
            ExceptionClass::DataAbort,
            &snapshot(ExceptionClass::DataAbort),
        );
        assert_eq!(
            recursive.classify(GENERATION),
            Err(RetainedError::Recursive)
        );

        let mut unknown = record;
        unknown.capture_raw(99, &snapshot(ExceptionClass::Undefined));
        assert_eq!(
            unknown.classify(GENERATION),
            Err(RetainedError::UnknownClass)
        );
    }

    #[test]
    fn wrong_mode_thumb_or_little_endian_context_is_rejected() {
        for mutation in 0..3 {
            let mut captured = snapshot(ExceptionClass::Undefined);
            match mutation {
                0 => captured.entry_cpsr = CPSR_E | MODE_ABORT,
                1 => captured.source_spsr |= CPSR_T,
                2 => captured.source_spsr &= !CPSR_E,
                _ => unreachable!(),
            }
            let mut record = RetainedExceptionRecord::EMPTY;
            record.arm(GENERATION);
            record.capture(ExceptionClass::Undefined, &captured);
            assert_eq!(
                record.classify(GENERATION),
                Err(RetainedError::UnsupportedState)
            );
        }

        let mut inconsistent_flags = RetainedExceptionRecord::EMPTY;
        inconsistent_flags.arm(GENERATION);
        inconsistent_flags.capture(
            ExceptionClass::Undefined,
            &snapshot(ExceptionClass::Undefined),
        );
        inconsistent_flags.words[INDEX_FLAGS] |= FLAG_IFAR_VALID;
        inconsistent_flags.seal(COMMIT_COMPLETE);
        assert_eq!(
            inconsistent_flags.classify(GENERATION),
            Err(RetainedError::Corrupt)
        );
    }

    #[test]
    fn record_size_and_crc_cover_target_be32_bytes() {
        let mut record = RetainedExceptionRecord::EMPTY;
        record.arm(GENERATION);
        assert_eq!(size_of::<RetainedExceptionRecord>(), 160);
        assert_eq!(record.words[INDEX_CRC], 0xe503_15c0);
        assert_eq!(record.words[INDEX_CRC_INVERSE], !0xe503_15c0);
    }
}
