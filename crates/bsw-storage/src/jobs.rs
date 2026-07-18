//! Poll-based asynchronous storage job layer.
//!
//! Upstream OpenBSW queues `storage::StorageJob` objects into a
//! `QueuingStorage` and reports completion through an
//! `etl::delegate` callback. The Rust port replaces callbacks with
//! polling, the native ownership-friendly shape: a caller submits a
//! [`StorageJob`] describing a read, write, or invalidate with
//! caller-owned buffers, receives a generation-validated [`JobHandle`]
//! (the same scheme as `bsw_time::TimerHandle`), and observes completion
//! via [`JobQueue::poll`] — no callback registration, no re-entrancy, and
//! completion state that survives until explicitly consumed with
//! [`JobQueue::take`].
//!
//! [`JobQueue::process`] drains at most `budget` jobs per call against any
//! [`BlockStore`], so the storage task can bound its per-cycle work.

use crate::backend::StorageError;
use crate::block::{BlockId, BlockStore};

/// Descriptor of one asynchronous storage operation.
///
/// Buffers are caller-owned borrows with lifetime `'buf`; the queue holds
/// them only while the job is enqueued or its result is pending.
pub enum StorageJob<'buf> {
    /// Read the payload of `id` into `buf`; the job result is the payload
    /// length.
    Read {
        /// Block to read.
        id: BlockId,
        /// Destination buffer, at least as long as the stored payload.
        buf: &'buf mut [u8],
    },
    /// Atomically write `data` as the new payload of `id`; the job result
    /// is the number of bytes written.
    Write {
        /// Block to write.
        id: BlockId,
        /// Application-defined payload format version.
        version: u16,
        /// Payload bytes.
        data: &'buf [u8],
    },
    /// Invalidate `id`; the job result is 0.
    Invalidate {
        /// Block to invalidate.
        id: BlockId,
    },
}

/// Handle to a submitted job, validated by slot generation.
///
/// A handle from a previous occupancy of a slot (already taken or
/// cancelled) is detected via the generation counter and reported as
/// [`JobState::Unknown`], never misattributed to a newer job.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JobHandle {
    index: u16,
    generation: u16,
}

/// Observable state of a submitted job.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JobState {
    /// The job is waiting for [`JobQueue::process`].
    Queued,
    /// The job executed; the payload is the operation result (bytes read
    /// or written, 0 for invalidate).
    Done(Result<usize, StorageError>),
    /// The handle does not refer to a live job (stale generation, already
    /// taken, or cancelled).
    Unknown,
}

enum Phase<'buf> {
    Queued(StorageJob<'buf>),
    Done(Result<usize, StorageError>),
}

struct Slot<'buf> {
    seq: u64,
    phase: Phase<'buf>,
}

/// Fixed-capacity job queue with budgeted processing.
///
/// `N` is the maximum number of in-flight jobs (queued or completed but
/// not yet taken). The queue never allocates.
pub struct JobQueue<'buf, const N: usize> {
    slots: [Option<Slot<'buf>>; N],
    generations: [u16; N],
    next_seq: u64,
}

impl<'buf, const N: usize> JobQueue<'buf, N> {
    /// Create an empty queue.
    pub const fn new() -> Self {
        Self {
            slots: [const { None }; N],
            generations: [0; N],
            next_seq: 0,
        }
    }

    /// Number of occupied slots (queued plus completed-but-not-taken).
    pub fn len(&self) -> usize {
        self.slots.iter().filter(|slot| slot.is_some()).count()
    }

    /// `true` when no slot is occupied.
    pub fn is_empty(&self) -> bool {
        self.slots.iter().all(Option::is_none)
    }

    /// Maximum number of in-flight jobs.
    pub const fn capacity(&self) -> usize {
        N
    }

    /// Submit a job for later processing.
    ///
    /// Fails with [`StorageError::Busy`] when all `N` slots are occupied.
    pub fn submit(&mut self, job: StorageJob<'buf>) -> Result<JobHandle, StorageError> {
        let index = self
            .slots
            .iter()
            .position(Option::is_none)
            .ok_or(StorageError::Busy)?;
        self.generations[index] = self.generations[index].wrapping_add(1);
        let seq = self.next_seq;
        self.next_seq = self.next_seq.wrapping_add(1);
        self.slots[index] = Some(Slot {
            seq,
            phase: Phase::Queued(job),
        });
        Ok(JobHandle {
            index: index as u16,
            generation: self.generations[index],
        })
    }

    /// Execute up to `budget` queued jobs against `store`, oldest first.
    ///
    /// Returns the number of jobs executed. Results are retained in the
    /// queue until observed with [`poll`](Self::poll) and consumed with
    /// [`take`](Self::take).
    pub fn process<S: BlockStore>(&mut self, store: &mut S, budget: usize) -> usize {
        let mut executed = 0;
        while executed < budget {
            let Some(index) = self.oldest_queued() else {
                break;
            };
            let Some(slot) = self.slots[index].as_mut() else {
                break;
            };
            let Phase::Queued(job) = &mut slot.phase else {
                break;
            };
            let result = Self::execute(store, job);
            slot.phase = Phase::Done(result);
            executed += 1;
        }
        executed
    }

    /// Observe the state of a job without consuming it.
    pub fn poll(&self, handle: JobHandle) -> JobState {
        match self.live_slot(handle) {
            Some(slot) => match &slot.phase {
                Phase::Queued(_) => JobState::Queued,
                Phase::Done(result) => JobState::Done(*result),
            },
            None => JobState::Unknown,
        }
    }

    /// Consume a completed job, freeing its slot and releasing its buffer
    /// borrow.
    ///
    /// Returns `None` when the handle is stale or the job has not executed
    /// yet (queued jobs must be [`cancel`](Self::cancel)led instead).
    pub fn take(&mut self, handle: JobHandle) -> Option<Result<usize, StorageError>> {
        let index = usize::from(handle.index);
        let result = match self.live_slot(handle)?.phase {
            Phase::Done(result) => Some(result),
            Phase::Queued(_) => None,
        }?;
        self.slots[index] = None;
        Some(result)
    }

    /// Remove a job that is still queued.
    ///
    /// Returns `true` when the job was cancelled; `false` when the handle
    /// is stale or the job already executed.
    pub fn cancel(&mut self, handle: JobHandle) -> bool {
        let index = usize::from(handle.index);
        let is_queued = self
            .live_slot(handle)
            .is_some_and(|slot| matches!(slot.phase, Phase::Queued(_)));
        if is_queued {
            self.slots[index] = None;
        }
        is_queued
    }

    fn live_slot(&self, handle: JobHandle) -> Option<&Slot<'buf>> {
        let index = usize::from(handle.index);
        if index >= N || self.generations[index] != handle.generation {
            return None;
        }
        self.slots[index].as_ref()
    }

    fn oldest_queued(&self) -> Option<usize> {
        self.slots
            .iter()
            .enumerate()
            .filter_map(|(index, slot)| match slot {
                Some(Slot {
                    seq,
                    phase: Phase::Queued(_),
                }) => Some((index, *seq)),
                _ => None,
            })
            .min_by_key(|&(_, seq)| seq)
            .map(|(index, _)| index)
    }

    fn execute<S: BlockStore>(
        store: &mut S,
        job: &mut StorageJob<'_>,
    ) -> Result<usize, StorageError> {
        match job {
            StorageJob::Read { id, buf } => store.read(*id, buf),
            StorageJob::Write { id, version, data } => {
                store.write(*id, *version, data).map(|()| data.len())
            }
            StorageJob::Invalidate { id } => store.invalidate(*id).map(|()| 0),
        }
    }
}

impl<const N: usize> Default for JobQueue<'_, N> {
    fn default() -> Self {
        Self::new()
    }
}
