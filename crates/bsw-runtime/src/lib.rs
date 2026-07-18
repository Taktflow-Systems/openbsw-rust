//! Runtime statistics and execution monitoring for embedded BSW.
//!
//! Ports the C++ `OpenBSW` `runtime` module to `no_std`-compatible Rust.
//! All timing is expressed in abstract u64 ticks — no clock dependency.
#![cfg_attr(not(feature = "std"), no_std)]
#![warn(clippy::all, clippy::pedantic)]

// ── RuntimeStatistics ────────────────────────────────────────────────────────

/// Tracks min/max/average for a series of duration measurements.
///
/// Semantics mirror upstream `runtime::RuntimeStatistics`: an empty tracker
/// reports zero for every value, and the first measurement initializes both
/// minimum and maximum. Totals saturate instead of wrapping — the upstream
/// `uint32_t` counters wrap silently, which is recorded as an intentional
/// native difference in the parity manifest.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RuntimeStatistics {
    count: u64,
    total: u64,
    min: u64,
    max: u64,
}

impl Default for RuntimeStatistics {
    fn default() -> Self {
        Self::new()
    }
}

impl RuntimeStatistics {
    /// Create a new, empty statistics tracker.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            count: 0,
            total: 0,
            min: 0,
            max: 0,
        }
    }

    /// Record a new measurement.
    pub fn record(&mut self, value: u64) {
        self.total = self.total.saturating_add(value);
        if self.count == 0 {
            self.min = value;
            self.max = value;
        } else {
            if value < self.min {
                self.min = value;
            }
            if value > self.max {
                self.max = value;
            }
        }
        self.count = self.count.saturating_add(1);
    }

    /// Number of recorded measurements.
    #[must_use]
    pub const fn count(&self) -> u64 {
        self.count
    }

    /// Minimum recorded value (0 if no measurements have been taken).
    #[must_use]
    pub const fn min(&self) -> u64 {
        self.min
    }

    /// Maximum recorded value (0 if no measurements have been taken).
    #[must_use]
    pub const fn max(&self) -> u64 {
        self.max
    }

    /// Average value, truncated toward zero. Returns 0 if no measurements.
    #[must_use]
    pub const fn average(&self) -> u64 {
        if self.count == 0 {
            0
        } else {
            self.total / self.count
        }
    }

    /// Total sum of all measurements.
    #[must_use]
    pub const fn total(&self) -> u64 {
        self.total
    }

    /// Reset all statistics to their initial state.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Return a snapshot of the current values and reset this tracker.
    ///
    /// This is the deterministic reporting cycle used by monitors: the
    /// returned copy is stable while new measurements start from zero.
    #[must_use]
    pub fn take(&mut self) -> Self {
        core::mem::take(self)
    }
}

// ── FunctionRuntimeStatistics ─────────────────────────────────────────────────

/// Extended statistics that also track jitter.
///
/// Jitter follows upstream `runtime::FunctionRuntimeStatistics`: it is the
/// interval between the start timestamps of consecutive runs, recorded from
/// the second run onward, not the variation of the runtimes themselves.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FunctionRuntimeStatistics {
    runtime: RuntimeStatistics,
    jitter: RuntimeStatistics,
    prev_start: Option<u64>,
}

impl Default for FunctionRuntimeStatistics {
    fn default() -> Self {
        Self::new()
    }
}

impl FunctionRuntimeStatistics {
    /// Create a new, empty tracker.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            runtime: RuntimeStatistics::new(),
            jitter: RuntimeStatistics::new(),
            prev_start: None,
        }
    }

    /// Record one run that started at `start_timestamp` ticks and consumed
    /// `runtime` ticks. Jitter is the wrapping interval between consecutive
    /// start timestamps.
    pub fn record(&mut self, start_timestamp: u64, runtime: u64) {
        if let Some(prev) = self.prev_start {
            self.jitter.record(start_timestamp.wrapping_sub(prev));
        }
        self.prev_start = Some(start_timestamp);
        self.runtime.record(runtime);
    }

    /// Access the runtime statistics.
    #[must_use]
    pub const fn runtime(&self) -> &RuntimeStatistics {
        &self.runtime
    }

    /// Access the jitter statistics.
    #[must_use]
    pub const fn jitter(&self) -> &RuntimeStatistics {
        &self.jitter
    }

    /// Minimum start-interval jitter; upstream `getMinJitter`.
    #[must_use]
    pub const fn min_jitter(&self) -> u64 {
        self.jitter.min()
    }

    /// Maximum start-interval jitter; upstream `getMaxJitter`.
    #[must_use]
    pub const fn max_jitter(&self) -> u64 {
        self.jitter.max()
    }

    /// Reset both runtime and jitter statistics, including the last start
    /// timestamp.
    pub fn reset(&mut self) {
        self.runtime.reset();
        self.jitter.reset();
        self.prev_start = None;
    }

    /// Return a snapshot of the current values and reset this tracker.
    #[must_use]
    pub fn take(&mut self) -> Self {
        core::mem::take(self)
    }
}

// ── ContextType ───────────────────────────────────────────────────────────────

/// Execution context type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContextType {
    /// Regular task (cooperative or preemptive scheduler task).
    Task,
    /// Interrupt service routine.
    Isr,
    /// Inline function / named code section.
    Function,
}

// ── StackEntry ────────────────────────────────────────────────────────────────

/// One entry in the [`RuntimeStack`].
#[derive(Debug, Clone)]
pub struct StackEntry {
    context: ContextType,
    /// Tick at which this entry most recently became active (either first push
    /// or last resume).
    start_tick: u64,
    /// Accumulated active ticks from all previous active segments (before the
    /// most recent suspend/preemption).
    accumulated_ticks: u64,
}

impl StackEntry {
    const fn new(context: ContextType, start_tick: u64) -> Self {
        Self {
            context,
            start_tick,
            accumulated_ticks: 0,
        }
    }
}

// ── RuntimeStack ──────────────────────────────────────────────────────────────

/// Stack tracking nested execution contexts.
///
/// When an ISR fires during a task the task's entry is suspended.
/// When the ISR completes the suspended time is added back, so the task's
/// actual runtime does not include the ISR time.
///
/// `MAX_DEPTH` is a compile-time constant controlling the maximum nesting
/// depth.
pub struct RuntimeStack<const MAX_DEPTH: usize> {
    entries: [Option<StackEntry>; MAX_DEPTH],
    depth: usize,
}

impl<const MAX_DEPTH: usize> Default for RuntimeStack<MAX_DEPTH> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const MAX_DEPTH: usize> RuntimeStack<MAX_DEPTH> {
    /// Create an empty stack.
    #[must_use]
    pub const fn new() -> Self {
        const NONE: Option<StackEntry> = None;
        Self {
            entries: [NONE; MAX_DEPTH],
            depth: 0,
        }
    }

    /// Push a new execution context.
    ///
    /// Suspends the current top entry so its elapsed time is frozen until it
    /// is resumed.  Returns the new depth index, or `None` if the stack is
    /// already full.
    pub fn push(&mut self, context: ContextType, current_tick: u64) -> Option<usize> {
        if self.depth >= MAX_DEPTH {
            return None;
        }

        // Suspend the current top: accumulate the active segment that just
        // ended, and remember it for when this entry is resumed.
        if self.depth > 0 {
            if let Some(top) = self.entries[self.depth - 1].as_mut() {
                top.accumulated_ticks += current_tick.saturating_sub(top.start_tick);
                // start_tick is now stale until the entry is resumed.
            }
        }

        self.entries[self.depth] = Some(StackEntry::new(context, current_tick));
        let new_depth = self.depth;
        self.depth += 1;
        Some(new_depth)
    }

    /// Pop the current execution context.
    ///
    /// Returns the net runtime (accumulated active time + current segment) for
    /// the popped entry, or `None` if the stack is empty.  Resumes the new
    /// top entry.
    pub fn pop(&mut self, current_tick: u64) -> Option<u64> {
        if self.depth == 0 {
            return None;
        }

        self.depth -= 1;
        let entry = self.entries[self.depth].take()?;

        // Net = all previously accumulated active segments + current segment.
        let last_segment = current_tick.saturating_sub(entry.start_tick);
        let net = entry.accumulated_ticks + last_segment;

        // Resume the new top: reset its start_tick so its next elapsed
        // calculation begins from now.
        if self.depth > 0 {
            if let Some(top) = self.entries[self.depth - 1].as_mut() {
                top.start_tick = current_tick;
            }
        }

        Some(net)
    }

    /// Current stack depth (number of entries on the stack).
    #[must_use]
    pub const fn depth(&self) -> usize {
        self.depth
    }

    /// Whether the stack is empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.depth == 0
    }

    /// Peek at the current (top) context type, or `None` if the stack is
    /// empty.
    #[must_use]
    pub fn current_context(&self) -> Option<ContextType> {
        if self.depth == 0 {
            return None;
        }
        self.entries[self.depth - 1].as_ref().map(|e| e.context)
    }
}

// ── ExecutionMonitor ──────────────────────────────────────────────────────────

/// Monitors execution time for a named function/section.
///
/// Usage: call [`enter`](ExecutionMonitor::enter) at function entry and
/// [`exit`](ExecutionMonitor::exit) at function exit.  The monitor records the
/// net execution time, optionally subtracting time spent suspended (preempted).
pub struct ExecutionMonitor {
    name: &'static str,
    stats: FunctionRuntimeStatistics,
    enter_tick: Option<u64>,
}

impl ExecutionMonitor {
    /// Create a new monitor with the given name and zero statistics.
    #[must_use]
    pub const fn new(name: &'static str) -> Self {
        Self {
            name,
            stats: FunctionRuntimeStatistics::new(),
            enter_tick: None,
        }
    }

    /// Mark entry into the monitored section.
    pub fn enter(&mut self, current_tick: u64) {
        self.enter_tick = Some(current_tick);
    }

    /// Mark exit from the monitored section.
    ///
    /// Computes `elapsed = current_tick - enter_tick`, then subtracts
    /// `suspended_ticks` to obtain the net active time.  Does nothing if
    /// [`enter`](ExecutionMonitor::enter) was never called.
    pub fn exit(&mut self, current_tick: u64, suspended_ticks: u64) {
        if let Some(enter) = self.enter_tick.take() {
            let elapsed = current_tick.saturating_sub(enter);
            let net = elapsed.saturating_sub(suspended_ticks);
            self.stats.record(enter, net);
        }
    }

    /// Get the accumulated statistics.
    #[must_use]
    pub const fn stats(&self) -> &FunctionRuntimeStatistics {
        &self.stats
    }

    /// Get the monitor's name.
    #[must_use]
    pub const fn name(&self) -> &str {
        self.name
    }

    /// Reset all statistics (and clear any in-progress enter tick).
    pub fn reset(&mut self) {
        self.stats.reset();
        self.enter_tick = None;
    }
}

// ── StatisticsContainer ───────────────────────────────────────────────────────

/// Fixed-capacity container for named [`RuntimeStatistics`] entries.
///
/// `N` is a compile-time constant maximum number of entries.
pub struct StatisticsContainer<const N: usize> {
    entries: [Option<(&'static str, RuntimeStatistics)>; N],
    count: usize,
}

impl<const N: usize> Default for StatisticsContainer<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> StatisticsContainer<N> {
    /// Create an empty container.
    #[must_use]
    pub const fn new() -> Self {
        const NONE: Option<(&'static str, RuntimeStatistics)> = None;
        Self {
            entries: [NONE; N],
            count: 0,
        }
    }

    /// Add a named statistics entry.
    ///
    /// Returns the index of the new entry, or `None` if the container is full.
    pub fn add(&mut self, name: &'static str) -> Option<usize> {
        if self.count >= N {
            return None;
        }
        let idx = self.count;
        self.entries[idx] = Some((name, RuntimeStatistics::new()));
        self.count += 1;
        Some(idx)
    }

    /// Record a value for the entry at the given index.
    ///
    /// Does nothing if `index` is out of range or the slot is empty.
    pub fn record(&mut self, index: usize, value: u64) {
        if let Some(Some((_, stats))) = self.entries.get_mut(index) {
            stats.record(value);
        }
    }

    /// Get statistics by index.
    ///
    /// Returns `Some((&name, &stats))` or `None` if out of range / empty.
    #[must_use]
    pub fn get(&self, index: usize) -> Option<(&str, &RuntimeStatistics)> {
        self.entries
            .get(index)?
            .as_ref()
            .map(|(name, stats)| (*name, stats))
    }

    /// Find statistics by name (linear scan).
    ///
    /// Returns `None` if no entry with the given name exists.
    #[must_use]
    pub fn find(&self, name: &str) -> Option<&RuntimeStatistics> {
        self.entries[..self.count]
            .iter()
            .find_map(|slot| slot.as_ref().filter(|(n, _)| *n == name).map(|(_, s)| s))
    }

    /// Number of entries currently stored.
    #[must_use]
    pub const fn count(&self) -> usize {
        self.count
    }

    /// Iterate over all entries as `(&str, &RuntimeStatistics)` pairs.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &RuntimeStatistics)> {
        self.entries[..self.count]
            .iter()
            .filter_map(|slot| slot.as_ref().map(|(n, s)| (*n, s)))
    }

    /// Reset all entries to their initial state (counts/totals/min/max
    /// cleared; names retained).
    pub fn reset_all(&mut self) {
        for slot in &mut self.entries[..self.count] {
            if let Some((_, stats)) = slot.as_mut() {
                stats.reset();
            }
        }
    }

    /// Copy names and statistics from `src`, mirroring upstream
    /// `StatisticsContainer::copyFrom` / `SharedStatisticsContainer`.
    pub fn copy_from(&mut self, src: &Self) {
        self.entries.clone_from(&src.entries);
        self.count = src.count;
    }

    /// Snapshot `src` into this container and reset `src` for the next
    /// measurement window.
    pub fn take_from(&mut self, src: &mut Self) {
        self.copy_from(src);
        src.reset_all();
    }
}

// ── FunctionStatisticsContainer ───────────────────────────────────────────────

/// Fixed-capacity container for named [`FunctionRuntimeStatistics`] entries.
///
/// This is the task/function monitoring companion of
/// [`StatisticsContainer`]: each entry additionally tracks start-interval
/// jitter, matching upstream containers instantiated with
/// `FunctionRuntimeStatistics`.
pub struct FunctionStatisticsContainer<const N: usize> {
    entries: [Option<(&'static str, FunctionRuntimeStatistics)>; N],
    count: usize,
}

impl<const N: usize> Default for FunctionStatisticsContainer<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> FunctionStatisticsContainer<N> {
    /// Create an empty container.
    #[must_use]
    pub const fn new() -> Self {
        const NONE: Option<(&'static str, FunctionRuntimeStatistics)> = None;
        Self {
            entries: [NONE; N],
            count: 0,
        }
    }

    /// Add a named entry. Returns its index, or `None` when full.
    pub fn add(&mut self, name: &'static str) -> Option<usize> {
        if self.count >= N {
            return None;
        }
        let idx = self.count;
        self.entries[idx] = Some((name, FunctionRuntimeStatistics::new()));
        self.count += 1;
        Some(idx)
    }

    /// Record one run for the entry at `index`.
    ///
    /// Does nothing if `index` is out of range or the slot is empty.
    pub fn record(&mut self, index: usize, start_timestamp: u64, runtime: u64) {
        if let Some(Some((_, stats))) = self.entries.get_mut(index) {
            stats.record(start_timestamp, runtime);
        }
    }

    /// Get statistics by index.
    #[must_use]
    pub fn get(&self, index: usize) -> Option<(&str, &FunctionRuntimeStatistics)> {
        self.entries
            .get(index)?
            .as_ref()
            .map(|(name, stats)| (*name, stats))
    }

    /// Find statistics by name (linear scan).
    #[must_use]
    pub fn find(&self, name: &str) -> Option<&FunctionRuntimeStatistics> {
        self.entries[..self.count]
            .iter()
            .find_map(|slot| slot.as_ref().filter(|(n, _)| *n == name).map(|(_, s)| s))
    }

    /// Number of entries currently stored.
    #[must_use]
    pub const fn count(&self) -> usize {
        self.count
    }

    /// Iterate over all entries as `(&str, &FunctionRuntimeStatistics)`.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &FunctionRuntimeStatistics)> {
        self.entries[..self.count]
            .iter()
            .filter_map(|slot| slot.as_ref().map(|(n, s)| (*n, s)))
    }

    /// Reset all entries; names are retained.
    pub fn reset_all(&mut self) {
        for slot in &mut self.entries[..self.count] {
            if let Some((_, stats)) = slot.as_mut() {
                stats.reset();
            }
        }
    }

    /// Copy names and statistics from `src`.
    pub fn copy_from(&mut self, src: &Self) {
        self.entries.clone_from(&src.entries);
        self.count = src.count;
    }

    /// Snapshot `src` into this container and reset `src`.
    pub fn take_from(&mut self, src: &mut Self) {
        self.copy_from(src);
        src.reset_all();
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── RuntimeStatistics ────────────────────────────────────────────────────

    #[test]
    fn runtime_stats_new_initial_state() {
        let s = RuntimeStatistics::new();
        assert_eq!(s.count(), 0);
        assert_eq!(s.min(), 0);
        assert_eq!(s.max(), 0);
        assert_eq!(s.average(), 0);
        assert_eq!(s.total(), 0);
    }

    /// Port of upstream `RuntimeStatisticsTest.testAddRun`
    /// (`libs/bsw/runtime/test/src/RuntimeStatisticsTest.cpp`).
    #[test]
    fn upstream_runtime_statistics_add_run_vectors() {
        let mut s = RuntimeStatistics::new();
        s.record(15);
        assert_eq!(
            (s.total(), s.count(), s.min(), s.max(), s.average()),
            (15, 1, 15, 15, 15)
        );
        s.record(30);
        assert_eq!(
            (s.total(), s.count(), s.min(), s.max(), s.average()),
            (45, 2, 15, 30, 22)
        );
        s.record(10);
        assert_eq!(
            (s.total(), s.count(), s.min(), s.max(), s.average()),
            (55, 3, 10, 30, 18)
        );
        s.record(20);
        assert_eq!(
            (s.total(), s.count(), s.min(), s.max(), s.average()),
            (75, 4, 10, 30, 18)
        );
    }

    /// Port of upstream `RuntimeStatisticsTest.testReset`.
    #[test]
    fn upstream_runtime_statistics_reset_vectors() {
        let mut s = RuntimeStatistics::new();
        s.record(15);
        s.reset();
        assert_eq!(
            (s.total(), s.count(), s.min(), s.max(), s.average()),
            (0, 0, 0, 0, 0)
        );
    }

    #[test]
    fn runtime_stats_total_and_count_saturate_instead_of_wrapping() {
        let mut s = RuntimeStatistics::new();
        s.record(u64::MAX);
        s.record(1);
        assert_eq!(s.total(), u64::MAX);
        assert_eq!(s.count(), 2);
        assert_eq!(s.min(), 1);
        assert_eq!(s.max(), u64::MAX);
    }

    #[test]
    fn runtime_stats_take_returns_snapshot_and_resets() {
        let mut s = RuntimeStatistics::new();
        s.record(10);
        s.record(20);
        let snapshot = s.take();
        assert_eq!(snapshot.count(), 2);
        assert_eq!(snapshot.total(), 30);
        assert_eq!(s.count(), 0);
        assert_eq!(s.total(), 0);
    }

    /// Property-style sweep: for any pseudo-random sequence the invariants
    /// `min <= average <= max`, `count`, and `total` hold exactly.
    #[test]
    fn runtime_stats_random_sequence_invariants() {
        let mut seed: u64 = 0x2545_F491_4F6C_DD1D;
        for _ in 0..64 {
            let mut s = RuntimeStatistics::new();
            let mut expected_total: u64 = 0;
            let mut expected_min = u64::MAX;
            let mut expected_max = 0;
            let samples = 1 + (seed % 100);
            for _ in 0..samples {
                seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
                let value = seed >> 33;
                s.record(value);
                expected_total += value;
                expected_min = expected_min.min(value);
                expected_max = expected_max.max(value);
            }
            assert_eq!(s.count(), samples);
            assert_eq!(s.total(), expected_total);
            assert_eq!(s.min(), expected_min);
            assert_eq!(s.max(), expected_max);
            assert!(s.min() <= s.average() && s.average() <= s.max());
        }
    }

    #[test]
    fn runtime_stats_record_single() {
        let mut s = RuntimeStatistics::new();
        s.record(42);
        assert_eq!(s.count(), 1);
        assert_eq!(s.min(), 42);
        assert_eq!(s.max(), 42);
        assert_eq!(s.total(), 42);
        assert_eq!(s.average(), 42);
    }

    #[test]
    fn runtime_stats_record_multiple_tracks_min_max() {
        let mut s = RuntimeStatistics::new();
        s.record(10);
        s.record(30);
        s.record(20);
        assert_eq!(s.min(), 10);
        assert_eq!(s.max(), 30);
        assert_eq!(s.count(), 3);
    }

    #[test]
    fn runtime_stats_average_correct() {
        let mut s = RuntimeStatistics::new();
        s.record(10);
        s.record(20);
        s.record(30);
        // (10+20+30)/3 = 20
        assert_eq!(s.average(), 20);
    }

    #[test]
    fn runtime_stats_average_rounds_down() {
        let mut s = RuntimeStatistics::new();
        s.record(1);
        s.record(2);
        // (1+2)/2 = 1.5 → truncated to 1
        assert_eq!(s.average(), 1);
    }

    #[test]
    fn runtime_stats_reset_clears_all() {
        let mut s = RuntimeStatistics::new();
        s.record(100);
        s.reset();
        assert_eq!(s.count(), 0);
        assert_eq!(s.total(), 0);
        assert_eq!(s.min(), 0);
        assert_eq!(s.max(), 0);
        assert_eq!(s.average(), 0);
    }

    #[test]
    fn runtime_stats_total_is_sum() {
        let mut s = RuntimeStatistics::new();
        s.record(5);
        s.record(15);
        s.record(30);
        assert_eq!(s.total(), 50);
    }

    // ── FunctionRuntimeStatistics ─────────────────────────────────────────────

    #[test]
    fn func_stats_new_no_jitter() {
        let f = FunctionRuntimeStatistics::new();
        assert_eq!(f.jitter().count(), 0);
        assert_eq!(f.runtime().count(), 0);
        assert_eq!(f.min_jitter(), 0);
        assert_eq!(f.max_jitter(), 0);
    }

    #[test]
    fn func_stats_first_record_no_jitter() {
        let mut f = FunctionRuntimeStatistics::new();
        f.record(25, 100);
        assert_eq!(f.runtime().count(), 1);
        assert_eq!(f.jitter().count(), 0, "no jitter on first measurement");
    }

    /// Port of upstream `FunctionRuntimeStatisticsTest.testAddRun`
    /// (`libs/bsw/runtime/test/src/FunctionRuntimeStatisticsTest.cpp`):
    /// jitter is the interval between consecutive start timestamps.
    #[test]
    fn upstream_function_runtime_statistics_add_run_vectors() {
        let mut f = FunctionRuntimeStatistics::new();
        f.record(25, 15);
        assert_eq!(f.runtime().total(), 15);
        assert_eq!(f.runtime().count(), 1);
        assert_eq!((f.min_jitter(), f.max_jitter()), (0, 0));
        f.record(45, 30);
        assert_eq!(f.runtime().total(), 45);
        assert_eq!(f.runtime().average(), 22);
        assert_eq!((f.min_jitter(), f.max_jitter()), (20, 20));
        f.record(85, 10);
        assert_eq!(f.runtime().total(), 55);
        assert_eq!(f.runtime().average(), 18);
        assert_eq!((f.min_jitter(), f.max_jitter()), (20, 40));
        f.record(100, 20);
        assert_eq!(f.runtime().total(), 75);
        assert_eq!(f.runtime().count(), 4);
        assert_eq!(
            (f.runtime().min(), f.runtime().max(), f.runtime().average()),
            (10, 30, 18)
        );
        assert_eq!((f.min_jitter(), f.max_jitter()), (15, 40));
    }

    /// Port of upstream `FunctionRuntimeStatisticsTest.testReset`.
    #[test]
    fn func_stats_reset_clears_all() {
        let mut f = FunctionRuntimeStatistics::new();
        f.record(20, 14);
        f.record(45, 16);
        assert_eq!(f.runtime().total(), 30);
        assert_eq!(f.runtime().average(), 15);
        assert_eq!((f.min_jitter(), f.max_jitter()), (25, 25));
        f.reset();
        assert_eq!(f.runtime().count(), 0);
        assert_eq!(f.jitter().count(), 0);
        assert_eq!((f.min_jitter(), f.max_jitter()), (0, 0));
        // After reset, first record should again produce no jitter.
        f.record(200, 10);
        assert_eq!(f.jitter().count(), 0);
    }

    #[test]
    fn func_stats_jitter_handles_timestamp_wraparound() {
        let mut f = FunctionRuntimeStatistics::new();
        f.record(u64::MAX - 4, 1);
        f.record(5, 1);
        assert_eq!(f.jitter().count(), 1);
        assert_eq!(f.min_jitter(), 10);
    }

    #[test]
    fn func_stats_take_returns_snapshot_and_resets() {
        let mut f = FunctionRuntimeStatistics::new();
        f.record(0, 5);
        f.record(10, 7);
        let snapshot = f.take();
        assert_eq!(snapshot.runtime().count(), 2);
        assert_eq!(snapshot.min_jitter(), 10);
        assert_eq!(f.runtime().count(), 0);
        f.record(50, 1);
        assert_eq!(f.jitter().count(), 0, "prev start cleared by take()");
    }

    // ── RuntimeStack ─────────────────────────────────────────────────────────

    #[test]
    fn stack_new_is_empty() {
        let s = RuntimeStack::<4>::new();
        assert!(s.is_empty());
        assert_eq!(s.depth(), 0);
        assert_eq!(s.current_context(), None);
    }

    #[test]
    fn stack_push_increases_depth() {
        let mut s = RuntimeStack::<4>::new();
        s.push(ContextType::Task, 0);
        assert_eq!(s.depth(), 1);
        assert!(!s.is_empty());
    }

    #[test]
    fn stack_pop_returns_net_runtime() {
        let mut s = RuntimeStack::<4>::new();
        s.push(ContextType::Task, 100);
        let net = s.pop(200);
        assert_eq!(net, Some(100));
        assert!(s.is_empty());
    }

    #[test]
    fn stack_nested_isr_preempts_task_time_excluded() {
        // Scenario:
        //   tick 0   : push Task
        //   tick 10  : push ISR  (task ran 10 ticks before preemption)
        //   tick 30  : pop ISR   → ISR net = 20 ticks
        //   tick 50  : pop Task  → Task net = 10 (before ISR) + 20 (after) = 30 ticks
        let mut s = RuntimeStack::<4>::new();
        s.push(ContextType::Task, 0);
        s.push(ContextType::Isr, 10);

        let isr_net = s.pop(30);
        assert_eq!(isr_net, Some(20), "ISR ran for 20 ticks");

        let task_net = s.pop(50);
        assert_eq!(task_net, Some(30), "Task ran for 10 + 20 = 30 active ticks");
    }

    #[test]
    fn stack_pop_on_empty_returns_none() {
        let mut s = RuntimeStack::<4>::new();
        assert_eq!(s.pop(100), None);
    }

    #[test]
    fn stack_push_on_full_returns_none() {
        let mut s = RuntimeStack::<2>::new();
        assert!(s.push(ContextType::Task, 0).is_some());
        assert!(s.push(ContextType::Isr, 1).is_some());
        assert_eq!(s.push(ContextType::Function, 2), None);
        assert_eq!(s.depth(), 2);
    }

    #[test]
    fn stack_current_context_correct_type() {
        let mut s = RuntimeStack::<4>::new();
        s.push(ContextType::Task, 0);
        assert_eq!(s.current_context(), Some(ContextType::Task));
        s.push(ContextType::Isr, 5);
        assert_eq!(s.current_context(), Some(ContextType::Isr));
        s.pop(10);
        assert_eq!(s.current_context(), Some(ContextType::Task));
    }

    #[test]
    fn stack_double_nesting_task_isr_nested_isr() {
        // tick 0  : push Task
        // tick 10 : push ISR1        (task active 10 ticks, then suspended)
        // tick 15 : push ISR2        (ISR1 active 5 ticks, then suspended)
        // tick 20 : pop ISR2         → net 5 ticks
        // tick 35 : pop ISR1         → net 5 (before ISR2) + 15 (after) = 20 ticks
        // tick 60 : pop Task         → net 10 (before ISR1) + 25 (after) = 35 ticks
        let mut s = RuntimeStack::<4>::new();
        s.push(ContextType::Task, 0);
        s.push(ContextType::Isr, 10);
        s.push(ContextType::Isr, 15);

        let isr2 = s.pop(20);
        assert_eq!(isr2, Some(5), "ISR2: 5 active ticks");

        let isr1 = s.pop(35);
        assert_eq!(isr1, Some(20), "ISR1: 5 + 15 = 20 active ticks");

        let task = s.pop(60);
        assert_eq!(task, Some(35), "Task: 10 + 25 = 35 active ticks");
    }

    // ── ExecutionMonitor ─────────────────────────────────────────────────────

    #[test]
    fn monitor_new_zero_stats() {
        let m = ExecutionMonitor::new("foo");
        assert_eq!(m.stats().runtime().count(), 0);
        assert_eq!(m.name(), "foo");
    }

    #[test]
    fn monitor_enter_exit_records_runtime() {
        let mut m = ExecutionMonitor::new("bar");
        m.enter(100);
        m.exit(200, 0);
        assert_eq!(m.stats().runtime().count(), 1);
        assert_eq!(m.stats().runtime().min(), 100);
        assert_eq!(m.stats().runtime().max(), 100);
    }

    #[test]
    fn monitor_enter_exit_with_suspended_ticks_subtracts() {
        let mut m = ExecutionMonitor::new("baz");
        m.enter(0);
        // elapsed = 50, suspended = 20 → net = 30
        m.exit(50, 20);
        assert_eq!(m.stats().runtime().min(), 30);
    }

    #[test]
    fn monitor_multiple_cycles_accumulate_stats() {
        let mut m = ExecutionMonitor::new("qux");
        m.enter(0);
        m.exit(10, 0);
        m.enter(100);
        m.exit(130, 0);
        assert_eq!(m.stats().runtime().count(), 2);
        assert_eq!(m.stats().runtime().min(), 10);
        assert_eq!(m.stats().runtime().max(), 30);
    }

    // ── StatisticsContainer ───────────────────────────────────────────────────

    #[test]
    fn container_new_is_empty() {
        let c = StatisticsContainer::<4>::new();
        assert_eq!(c.count(), 0);
    }

    #[test]
    fn container_add_entries_by_name() {
        let mut c = StatisticsContainer::<4>::new();
        let i0 = c.add("task_a");
        let i1 = c.add("task_b");
        assert_eq!(i0, Some(0));
        assert_eq!(i1, Some(1));
        assert_eq!(c.count(), 2);
    }

    #[test]
    fn container_record_values_by_index() {
        let mut c = StatisticsContainer::<4>::new();
        let idx = c.add("task_a").unwrap();
        c.record(idx, 50);
        c.record(idx, 80);
        let (_, stats) = c.get(idx).unwrap();
        assert_eq!(stats.count(), 2);
        assert_eq!(stats.min(), 50);
        assert_eq!(stats.max(), 80);
    }

    #[test]
    fn container_find_by_name() {
        let mut c = StatisticsContainer::<4>::new();
        c.add("alpha").unwrap();
        c.add("beta").unwrap();
        c.record(0, 10);
        let stats = c.find("alpha").unwrap();
        assert_eq!(stats.count(), 1);
        assert!(c.find("gamma").is_none());
    }

    #[test]
    fn container_add_beyond_capacity_returns_none() {
        let mut c = StatisticsContainer::<2>::new();
        assert!(c.add("a").is_some());
        assert!(c.add("b").is_some());
        assert_eq!(c.add("c"), None);
        assert_eq!(c.count(), 2);
    }

    #[test]
    fn container_reset_all_clears_entries() {
        let mut c = StatisticsContainer::<4>::new();
        let idx = c.add("x").unwrap();
        c.record(idx, 99);
        c.reset_all();
        let (name, stats) = c.get(idx).unwrap();
        assert_eq!(name, "x");
        assert_eq!(stats.count(), 0);
    }

    #[test]
    fn container_iter_yields_all_entries() {
        let mut c = StatisticsContainer::<4>::new();
        c.add("one").unwrap();
        c.add("two").unwrap();
        c.add("three").unwrap();
        let names: &[&str] = &["one", "two", "three"];
        for (i, (name, _)) in c.iter().enumerate() {
            assert_eq!(name, names[i]);
        }
        assert_eq!(c.iter().count(), 3);
    }

    #[test]
    fn container_copy_from_mirrors_names_and_values() {
        let mut src = StatisticsContainer::<4>::new();
        let idx = src.add("task_a").unwrap();
        src.record(idx, 42);
        let mut dst = StatisticsContainer::<4>::new();
        dst.copy_from(&src);
        let (name, stats) = dst.get(idx).unwrap();
        assert_eq!(name, "task_a");
        assert_eq!(stats.max(), 42);
        // Source keeps its values after a plain copy.
        assert_eq!(src.get(idx).unwrap().1.count(), 1);
    }

    #[test]
    fn container_take_from_snapshots_and_resets_source() {
        let mut src = StatisticsContainer::<4>::new();
        let idx = src.add("task_a").unwrap();
        src.record(idx, 7);
        let mut dst = StatisticsContainer::<4>::new();
        dst.take_from(&mut src);
        assert_eq!(dst.get(idx).unwrap().1.count(), 1);
        assert_eq!(src.get(idx).unwrap().1.count(), 0);
        assert_eq!(src.get(idx).unwrap().0, "task_a", "names survive reset");
    }

    // ── FunctionStatisticsContainer ──────────────────────────────────────────

    #[test]
    fn function_container_records_runtime_and_jitter() {
        let mut c = FunctionStatisticsContainer::<4>::new();
        let idx = c.add("cycle").unwrap();
        c.record(idx, 100, 10);
        c.record(idx, 150, 12);
        let (name, stats) = c.get(idx).unwrap();
        assert_eq!(name, "cycle");
        assert_eq!(stats.runtime().count(), 2);
        assert_eq!((stats.min_jitter(), stats.max_jitter()), (50, 50));
        assert_eq!(c.find("cycle").unwrap().runtime().max(), 12);
        assert!(c.find("absent").is_none());
    }

    #[test]
    fn function_container_capacity_reset_and_snapshot() {
        let mut c = FunctionStatisticsContainer::<1>::new();
        assert_eq!(c.add("only"), Some(0));
        assert_eq!(c.add("overflow"), None);
        c.record(0, 5, 9);
        let mut snapshot = FunctionStatisticsContainer::<1>::new();
        snapshot.take_from(&mut c);
        assert_eq!(snapshot.get(0).unwrap().1.runtime().total(), 9);
        assert_eq!(c.get(0).unwrap().1.runtime().count(), 0);
        assert_eq!(c.count(), 1);
        c.reset_all();
        assert_eq!(c.iter().count(), 1);
    }

    #[test]
    fn function_container_ignores_out_of_range_records() {
        let mut c = FunctionStatisticsContainer::<2>::new();
        c.add("a").unwrap();
        c.record(5, 0, 1);
        assert_eq!(c.get(0).unwrap().1.runtime().count(), 0);
    }
}
