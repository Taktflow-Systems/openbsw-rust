//! Runtime statistics and execution monitoring for embedded BSW.
//!
//! Ports the C++ `OpenBSW` `runtime` module to `no_std`-compatible Rust.
//! All timing is expressed in abstract u64 ticks — no clock dependency.
#![cfg_attr(not(feature = "std"), no_std)]
#![warn(clippy::all, clippy::pedantic)]

// ── RuntimeStatistics ────────────────────────────────────────────────────────

/// Tracks min/max/average for a series of duration measurements.
#[derive(Debug, Clone)]
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
            min: u64::MAX,
            max: 0,
        }
    }

    /// Record a new measurement.
    pub fn record(&mut self, value: u64) {
        self.count += 1;
        self.total += value;
        if value < self.min {
            self.min = value;
        }
        if value > self.max {
            self.max = value;
        }
    }

    /// Number of recorded measurements.
    #[must_use]
    pub const fn count(&self) -> u64 {
        self.count
    }

    /// Minimum recorded value (`u64::MAX` if no measurements have been taken).
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
}

// ── FunctionRuntimeStatistics ─────────────────────────────────────────────────

/// Extended statistics that also track jitter (variation between consecutive
/// measurements).
#[derive(Debug, Clone)]
pub struct FunctionRuntimeStatistics {
    runtime: RuntimeStatistics,
    jitter: RuntimeStatistics,
    last_value: Option<u64>,
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
            last_value: None,
        }
    }

    /// Record a new measurement. Also computes jitter (absolute difference
    /// from the previous measurement) when a previous value exists.
    pub fn record(&mut self, value: u64) {
        if let Some(prev) = self.last_value {
            self.jitter.record(value.abs_diff(prev));
        }
        self.last_value = Some(value);
        self.runtime.record(value);
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

    /// Reset both runtime and jitter statistics, including the last-seen value.
    pub fn reset(&mut self) {
        self.runtime.reset();
        self.jitter.reset();
        self.last_value = None;
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
        self.entries[self.depth - 1]
            .as_ref()
            .map(|e| e.context)
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
            self.stats.record(net);
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
        self.entries[..self.count].iter().find_map(|slot| {
            slot.as_ref()
                .filter(|(n, _)| *n == name)
                .map(|(_, s)| s)
        })
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
        assert_eq!(s.min(), u64::MAX);
        assert_eq!(s.max(), 0);
        assert_eq!(s.average(), 0);
        assert_eq!(s.total(), 0);
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
        assert_eq!(s.min(), u64::MAX);
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
    }

    #[test]
    fn func_stats_first_record_no_jitter() {
        let mut f = FunctionRuntimeStatistics::new();
        f.record(100);
        assert_eq!(f.runtime().count(), 1);
        assert_eq!(f.jitter().count(), 0, "no jitter on first measurement");
    }

    #[test]
    fn func_stats_second_record_jitter_equals_abs_diff() {
        let mut f = FunctionRuntimeStatistics::new();
        f.record(100);
        f.record(150);
        assert_eq!(f.jitter().count(), 1);
        assert_eq!(f.jitter().min(), 50);
        assert_eq!(f.jitter().max(), 50);
    }

    #[test]
    fn func_stats_multiple_records_track_jitter_min_max() {
        let mut f = FunctionRuntimeStatistics::new();
        f.record(100); // no jitter
        f.record(110); // jitter 10
        f.record(105); // jitter 5
        f.record(120); // jitter 15
        assert_eq!(f.jitter().min(), 5);
        assert_eq!(f.jitter().max(), 15);
        assert_eq!(f.jitter().count(), 3);
    }

    #[test]
    fn func_stats_reset_clears_all() {
        let mut f = FunctionRuntimeStatistics::new();
        f.record(50);
        f.record(80);
        f.reset();
        assert_eq!(f.runtime().count(), 0);
        assert_eq!(f.jitter().count(), 0);
        // After reset, first record should again produce no jitter.
        f.record(200);
        assert_eq!(f.jitter().count(), 0);
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
}
