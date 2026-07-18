//! Poll-based job queue over a journaled store.

use bsw_storage::jobs::{JobQueue, JobState, StorageJob};
use bsw_storage::journal::JournalStore;
use bsw_storage::mem::MemBackend;
use bsw_storage::{BlockId, BlockStore, StorageError};

type Backend = MemBackend<1024, 128, 4>;
type Store = JournalStore<Backend, 8>;

fn store() -> Store {
    JournalStore::mount(Backend::new()).expect("mount")
}

#[test]
fn write_job_then_read_job_round_trip() {
    let mut store = store();
    let data = *b"job-payload";
    let mut read_buf = [0u8; 32];
    {
        let mut queue: JobQueue<'_, 4> = JobQueue::new();
        let write = queue
            .submit(StorageJob::Write {
                id: BlockId(1),
                version: 2,
                data: &data,
            })
            .expect("submit write");
        assert_eq!(queue.poll(write), JobState::Queued);
        assert_eq!(queue.process(&mut store, 8), 1);
        assert_eq!(queue.poll(write), JobState::Done(Ok(data.len())));
        assert_eq!(queue.take(write), Some(Ok(data.len())));
        assert_eq!(
            queue.poll(write),
            JobState::Unknown,
            "taken handle is stale"
        );

        let read = queue
            .submit(StorageJob::Read {
                id: BlockId(1),
                buf: &mut read_buf,
            })
            .expect("submit read");
        assert_eq!(queue.process(&mut store, 8), 1);
        assert_eq!(queue.poll(read), JobState::Done(Ok(data.len())));
        assert_eq!(queue.take(read), Some(Ok(data.len())));
    }
    assert_eq!(&read_buf[..data.len()], &data);
}

#[test]
fn process_respects_budget_and_fifo_order() {
    let mut store = store();
    let first = *b"first";
    let second = *b"second";
    let third = *b"third";
    let mut queue: JobQueue<'_, 4> = JobQueue::new();
    let h1 = queue
        .submit(StorageJob::Write {
            id: BlockId(1),
            version: 1,
            data: &first,
        })
        .expect("submit 1");
    let h2 = queue
        .submit(StorageJob::Write {
            id: BlockId(2),
            version: 1,
            data: &second,
        })
        .expect("submit 2");
    let h3 = queue
        .submit(StorageJob::Write {
            id: BlockId(3),
            version: 1,
            data: &third,
        })
        .expect("submit 3");
    assert_eq!(
        queue.process(&mut store, 2),
        2,
        "budget bounds executed jobs"
    );
    assert_eq!(queue.poll(h1), JobState::Done(Ok(5)));
    assert_eq!(queue.poll(h2), JobState::Done(Ok(6)));
    assert_eq!(
        queue.poll(h3),
        JobState::Queued,
        "third job exceeds the budget"
    );
    assert_eq!(queue.process(&mut store, 2), 1);
    assert_eq!(queue.poll(h3), JobState::Done(Ok(5)));
    assert!(store.contains(BlockId(3)));
}

#[test]
fn queue_reports_busy_when_full() {
    let mut queue: JobQueue<'_, 2> = JobQueue::new();
    queue
        .submit(StorageJob::Invalidate { id: BlockId(1) })
        .expect("submit 1");
    queue
        .submit(StorageJob::Invalidate { id: BlockId(2) })
        .expect("submit 2");
    assert!(matches!(
        queue.submit(StorageJob::Invalidate { id: BlockId(3) }),
        Err(StorageError::Busy)
    ));
    assert_eq!(queue.len(), 2);
    assert_eq!(queue.capacity(), 2);
}

#[test]
fn cancel_and_stale_generation_detection() {
    let mut store = store();
    let mut queue: JobQueue<'_, 2> = JobQueue::new();
    let handle = queue
        .submit(StorageJob::Write {
            id: BlockId(1),
            version: 1,
            data: b"x",
        })
        .expect("submit");
    assert!(queue.cancel(handle), "queued job can be cancelled");
    assert_eq!(queue.poll(handle), JobState::Unknown);
    assert!(!queue.cancel(handle), "cancel is not repeatable");
    assert_eq!(
        queue.process(&mut store, 8),
        0,
        "cancelled job must not run"
    );
    assert!(!store.contains(BlockId(1)));

    // The slot is reused with a new generation; the old handle stays stale.
    let fresh = queue
        .submit(StorageJob::Write {
            id: BlockId(2),
            version: 1,
            data: b"y",
        })
        .expect("resubmit");
    assert_eq!(queue.poll(handle), JobState::Unknown);
    assert_eq!(queue.process(&mut store, 8), 1);
    assert_eq!(queue.take(fresh), Some(Ok(1)));
    assert!(queue.is_empty());
}

#[test]
fn job_errors_are_reported_via_poll() {
    let mut store = store();
    let mut buf = [0u8; 8];
    let mut queue: JobQueue<'_, 2> = JobQueue::new();
    let read = queue
        .submit(StorageJob::Read {
            id: BlockId(9),
            buf: &mut buf,
        })
        .expect("submit read");
    assert_eq!(queue.process(&mut store, 8), 1);
    assert_eq!(
        queue.poll(read),
        JobState::Done(Err(StorageError::UnknownBlock))
    );
    assert_eq!(queue.take(read), Some(Err(StorageError::UnknownBlock)));
}

#[test]
fn invalidate_job_runs_and_reports_zero() {
    let mut store = store();
    store.write(BlockId(4), 1, b"gone soon").expect("seed");
    let mut queue: JobQueue<'_, 2> = JobQueue::new();
    let handle = queue
        .submit(StorageJob::Invalidate { id: BlockId(4) })
        .expect("submit");
    assert_eq!(queue.process(&mut store, 1), 1);
    assert_eq!(queue.take(handle), Some(Ok(0)));
    assert!(!store.contains(BlockId(4)));
}

#[test]
fn take_before_processing_returns_none() {
    let mut queue: JobQueue<'_, 2> = JobQueue::new();
    let handle = queue
        .submit(StorageJob::Invalidate { id: BlockId(1) })
        .expect("submit");
    assert_eq!(queue.take(handle), None, "queued jobs cannot be taken");
    assert_eq!(queue.poll(handle), JobState::Queued);
}
