//! Process-wide reentrant critical section for POSIX hosts.
//!
//! On hardware, [`CriticalSection`] masks interrupts; on a host it must
//! provide mutual exclusion between threads standing in for interrupt and
//! task contexts. `std::sync::Mutex` alone is not reentrant (and
//! `ReentrantLock` is unstable), so [`PosixCriticalSection`] combines a
//! `Mutex`-guarded owner record with a thread-id recursion counter: the
//! first `acquire` on a thread takes ownership, nested `acquire`s bump a
//! depth counter, and the matching final `release` wakes one waiting
//! thread.

use std::sync::{Arc, Condvar, Mutex};
use std::thread::{self, ThreadId};

use bsw_platform::CriticalSection;

#[derive(Debug, Default)]
struct LockState {
    owner: Option<ThreadId>,
    depth: u32,
    unbalanced_releases: u32,
}

#[derive(Debug, Default)]
struct Inner {
    state: Mutex<LockState>,
    released: Condvar,
}

/// Reentrant, cloneable critical section implementing [`CriticalSection`].
///
/// Clones share the same underlying lock, so each thread can own its own
/// `&mut` handle (the trait takes `&mut self`) while still excluding the
/// others. Nesting works exactly like
/// `bsw_platform::mock::MockCriticalSection`: `acquire` nests, the section
/// ends when every acquire was released, and a `release` without a
/// matching `acquire` on the calling thread is counted in
/// [`Self::unbalanced_releases`] instead of corrupting the lock.
#[derive(Debug, Clone, Default)]
pub struct PosixCriticalSection {
    inner: Arc<Inner>,
}

impl PosixCriticalSection {
    /// Create a released critical section.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Current nesting depth (0 when no thread holds the section).
    #[must_use]
    pub fn depth(&self) -> u32 {
        self.inner
            .state
            .lock()
            .expect("critical-section mutex poisoned")
            .depth
    }

    /// Releases observed without a matching acquire by the calling thread
    /// (a caller bug, mirrored from the mock's bookkeeping).
    #[must_use]
    pub fn unbalanced_releases(&self) -> u32 {
        self.inner
            .state
            .lock()
            .expect("critical-section mutex poisoned")
            .unbalanced_releases
    }
}

impl CriticalSection for PosixCriticalSection {
    fn acquire(&mut self) {
        let me = thread::current().id();
        let mut state = self
            .inner
            .state
            .lock()
            .expect("critical-section mutex poisoned");
        if state.owner == Some(me) {
            state.depth += 1;
            return;
        }
        while state.owner.is_some() {
            state = self
                .inner
                .released
                .wait(state)
                .expect("critical-section mutex poisoned");
        }
        state.owner = Some(me);
        state.depth = 1;
    }

    fn release(&mut self) {
        let me = thread::current().id();
        let mut state = self
            .inner
            .state
            .lock()
            .expect("critical-section mutex poisoned");
        if state.owner != Some(me) {
            state.unbalanced_releases += 1;
            return;
        }
        state.depth -= 1;
        if state.depth == 0 {
            state.owner = None;
            drop(state);
            self.inner.released.notify_one();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bsw_platform::with_critical;

    #[test]
    fn nests_and_balances_on_one_thread() {
        let mut section = PosixCriticalSection::new();
        let value = with_critical(&mut section, || 7);
        assert_eq!(value, 7);
        assert_eq!(section.depth(), 0);
        section.acquire();
        section.acquire();
        assert_eq!(section.depth(), 2);
        section.release();
        assert_eq!(section.depth(), 1);
        section.release();
        assert_eq!(section.depth(), 0);
    }

    #[test]
    fn unmatched_release_is_counted_not_fatal() {
        let mut section = PosixCriticalSection::new();
        section.release();
        assert_eq!(section.unbalanced_releases(), 1);
        assert_eq!(section.depth(), 0);
        // The section still works normally afterwards.
        section.acquire();
        assert_eq!(section.depth(), 1);
        section.release();
        assert_eq!(section.depth(), 0);
        assert_eq!(section.unbalanced_releases(), 1);
    }

    #[test]
    fn clones_share_the_same_lock() {
        let mut a = PosixCriticalSection::new();
        let b = a.clone();
        a.acquire();
        assert_eq!(b.depth(), 1);
        a.release();
        assert_eq!(b.depth(), 0);
    }
}
