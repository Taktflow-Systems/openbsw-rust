//! Deterministic power-cut and torn-write fault injection.
//!
//! [`CutPointBackend`] wraps any [`StorageBackend`], counts every mutating
//! operation (`program` and `erase`), and can be armed to simulate a power
//! cut after exactly *K* operations. The K-th operation either fails
//! without touching the medium (a clean cut) or, for a program, writes
//! only a prefix of its bytes first (a torn write). Every mutating
//! operation after the cut fails with [`StorageError::Io`] and leaves the
//! medium untouched, modelling a device that lost power.
//!
//! Reads always succeed, so a test can drop the wrapped store, disarm the
//! wrapper, and re-mount to model the next power-on.

use crate::backend::{StorageBackend, StorageError};

/// Armed power-cut plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CutPlan {
    /// Number of mutating operations that complete normally before the
    /// cut. The operation with this (0-based) index is the cut operation.
    pub fail_after: u64,
    /// Torn-write behaviour of the cut operation when it is a `program`:
    /// `None` fails cleanly (nothing written); `Some(n)` writes the first
    /// `n` bytes (clamped to the operation length, rounded up to whole
    /// program units with `0xFF` filler so unwritten bytes stay erased)
    /// before failing. Erase operations always fail cleanly.
    pub torn_prefix: Option<usize>,
}

/// Fault-injecting wrapper around a [`StorageBackend`].
pub struct CutPointBackend<B: StorageBackend> {
    inner: B,
    ops: u64,
    plan: Option<CutPlan>,
    cut: bool,
}

impl<B: StorageBackend> CutPointBackend<B> {
    /// Wrap `inner` with fault injection disarmed.
    pub fn new(inner: B) -> Self {
        Self {
            inner,
            ops: 0,
            plan: None,
            cut: false,
        }
    }

    /// Arm a power cut and reset the operation counter.
    pub fn arm(&mut self, plan: CutPlan) {
        self.plan = Some(plan);
        self.ops = 0;
        self.cut = false;
    }

    /// Disarm fault injection; subsequent operations run normally.
    ///
    /// The operation counter keeps counting so a baseline run can measure
    /// the total mutating-operation count of a scenario.
    pub fn disarm(&mut self) {
        self.plan = None;
        self.cut = false;
    }

    /// Mutating operations observed since construction or the last
    /// [`arm`](Self::arm).
    pub fn op_count(&self) -> u64 {
        self.ops
    }

    /// `true` once the armed cut has triggered.
    pub fn cut_hit(&self) -> bool {
        self.cut
    }

    /// Unwrap the inner backend.
    pub fn into_inner(self) -> B {
        self.inner
    }

    /// Shared access to the inner backend.
    pub fn inner(&self) -> &B {
        &self.inner
    }

    /// Mutable access to the inner backend (test corruption hooks).
    pub fn inner_mut(&mut self) -> &mut B {
        &mut self.inner
    }

    /// Account one mutating operation. Returns `Err` when the cut has
    /// already happened, `Ok(None)` for a normal operation, and
    /// `Ok(Some(plan))` when this operation is the cut operation.
    fn account(&mut self) -> Result<Option<CutPlan>, StorageError> {
        if self.cut {
            return Err(StorageError::Io);
        }
        let index = self.ops;
        self.ops += 1;
        match self.plan {
            Some(plan) if index >= plan.fail_after => {
                self.cut = true;
                Ok(Some(plan))
            }
            _ => Ok(None),
        }
    }
}

impl<B: StorageBackend> StorageBackend for CutPointBackend<B> {
    fn region_size(&self) -> usize {
        self.inner.region_size()
    }

    fn erase_unit(&self) -> usize {
        self.inner.erase_unit()
    }

    fn program_unit(&self) -> usize {
        self.inner.program_unit()
    }

    fn read(&self, offset: usize, buf: &mut [u8]) -> Result<(), StorageError> {
        self.inner.read(offset, buf)
    }

    fn erase(&mut self, unit_index: usize) -> Result<(), StorageError> {
        match self.account()? {
            None => self.inner.erase(unit_index),
            Some(_) => Err(StorageError::Io),
        }
    }

    fn program(&mut self, offset: usize, data: &[u8]) -> Result<(), StorageError> {
        match self.account()? {
            None => self.inner.program(offset, data),
            Some(CutPlan {
                torn_prefix: Some(prefix),
                ..
            }) if prefix > 0 => {
                let unit = self.inner.program_unit();
                let torn = prefix.min(data.len());
                let programmed = torn.div_ceil(unit) * unit;
                let mut bytes = vec![0xFFu8; programmed];
                bytes[..torn].copy_from_slice(&data[..torn]);
                // Ignore a secondary error from the torn program itself:
                // the simulated outcome is Io either way.
                let _ = self.inner.program(offset, &bytes);
                Err(StorageError::Io)
            }
            Some(_) => Err(StorageError::Io),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mem::MemBackend;

    type Small = MemBackend<256, 64, 4>;

    #[test]
    fn disarmed_wrapper_counts_and_passes_through() {
        let mut cut = CutPointBackend::new(Small::new());
        cut.program(0, &[1, 2, 3, 4]).unwrap();
        cut.erase(0).unwrap();
        assert_eq!(cut.op_count(), 2);
        assert!(!cut.cut_hit());
    }

    #[test]
    fn clean_cut_fails_kth_and_all_later_ops() {
        let mut cut = CutPointBackend::new(Small::new());
        cut.arm(CutPlan {
            fail_after: 1,
            torn_prefix: None,
        });
        cut.program(0, &[1, 2, 3, 4]).unwrap();
        assert_eq!(cut.program(4, &[5, 6, 7, 8]), Err(StorageError::Io));
        assert!(cut.cut_hit());
        assert_eq!(cut.erase(0), Err(StorageError::Io));
        // The cut operation and everything after left the medium untouched.
        let mut buf = [0u8; 8];
        cut.read(0, &mut buf).unwrap();
        assert_eq!(buf, [1, 2, 3, 4, 0xFF, 0xFF, 0xFF, 0xFF]);
    }

    #[test]
    fn cut_at_zero_fails_first_op() {
        let mut cut = CutPointBackend::new(Small::new());
        cut.arm(CutPlan {
            fail_after: 0,
            torn_prefix: None,
        });
        assert_eq!(cut.program(0, &[1, 2, 3, 4]), Err(StorageError::Io));
    }

    #[test]
    fn torn_program_writes_prefix_and_keeps_rest_erased() {
        let mut cut = CutPointBackend::new(Small::new());
        cut.arm(CutPlan {
            fail_after: 0,
            torn_prefix: Some(3),
        });
        assert_eq!(
            cut.program(0, &[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]),
            Err(StorageError::Io)
        );
        let mut buf = [0u8; 8];
        cut.read(0, &mut buf).unwrap();
        assert_eq!(buf, [0x11, 0x22, 0x33, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);
    }

    #[test]
    fn torn_prefix_clamps_to_full_length() {
        let mut cut = CutPointBackend::new(Small::new());
        cut.arm(CutPlan {
            fail_after: 0,
            torn_prefix: Some(usize::MAX),
        });
        // Ack-lost scenario: the data is fully durable, the caller sees Io.
        assert_eq!(cut.program(0, &[1, 2, 3, 4]), Err(StorageError::Io));
        let mut buf = [0u8; 4];
        cut.read(0, &mut buf).unwrap();
        assert_eq!(buf, [1, 2, 3, 4]);
    }

    #[test]
    fn torn_erase_fails_cleanly() {
        let mut cut = CutPointBackend::new(Small::new());
        cut.program(0, &[1, 2, 3, 4]).unwrap();
        cut.arm(CutPlan {
            fail_after: 0,
            torn_prefix: Some(2),
        });
        assert_eq!(cut.erase(0), Err(StorageError::Io));
        let mut buf = [0u8; 4];
        cut.read(0, &mut buf).unwrap();
        assert_eq!(buf, [1, 2, 3, 4]);
    }

    #[test]
    fn disarm_restores_operation() {
        let mut cut = CutPointBackend::new(Small::new());
        cut.arm(CutPlan {
            fail_after: 0,
            torn_prefix: None,
        });
        assert_eq!(cut.program(0, &[1, 2, 3, 4]), Err(StorageError::Io));
        cut.disarm();
        cut.program(0, &[1, 2, 3, 4]).unwrap();
    }
}
