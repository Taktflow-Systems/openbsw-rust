//! Bounded log buffering with explicit overflow accounting (package C14).
//!
//! [`BufferedLogger`] renders every accepted statement into a fixed-size
//! entry at the call site and stores it in a ring. The caller therefore does
//! bounded work regardless of how slow — or absent — the draining sink is:
//! a full ring drops records and counts them instead of blocking.

use bsw_time::Clock;

use crate::{ComponentFilter, ComponentId, Level, LevelFilter, Log, Record};

/// What to do with a new record when the ring is full.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OverflowPolicy {
    /// Keep buffered history; count and discard the new record.
    DropNewest,
    /// Keep the newest records; count and discard the oldest.
    DropOldest,
}

/// Receiver for drained records.
pub trait Sink {
    /// Consume one drained record. `truncated` reports that the rendered
    /// message did not fit the entry buffer and was cut at a character
    /// boundary.
    fn consume(&mut self, record: &Record<'_>, truncated: bool);
}

/// `core::fmt::Write` adapter that truncates at character boundaries.
struct EntryWriter<'a> {
    bytes: &'a mut [u8],
    len: usize,
    truncated: bool,
}

impl core::fmt::Write for EntryWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        if self.truncated {
            // Never append later fragments after a cut: the message would
            // silently lose its middle instead of its tail.
            return Ok(());
        }
        let available = self.bytes.len() - self.len;
        let cut = if s.len() <= available {
            s.len()
        } else {
            self.truncated = true;
            floor_char_boundary(s, available)
        };
        self.bytes[self.len..self.len + cut].copy_from_slice(&s.as_bytes()[..cut]);
        self.len += cut;
        Ok(())
    }
}

/// Largest index `<= at` that lies on a character boundary.
fn floor_char_boundary(s: &str, at: usize) -> usize {
    let mut index = at.min(s.len());
    while index > 0 && !s.is_char_boundary(index) {
        index -= 1;
    }
    index
}

#[derive(Clone, Copy)]
struct Entry<const MSG: usize> {
    component: ComponentId,
    level: Level,
    timestamp: bsw_time::Instant,
    len: usize,
    truncated: bool,
    bytes: [u8; MSG],
}

impl<const MSG: usize> Entry<MSG> {
    const EMPTY: Self = Self {
        component: ComponentId::new(0),
        level: Level::Debug,
        timestamp: bsw_time::Instant::from_nanos(0),
        len: 0,
        truncated: false,
        bytes: [0; MSG],
    };

    fn text(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len]).unwrap_or("<invalid utf-8>")
    }
}

/// Fixed-capacity buffering logger.
///
/// `COMPONENTS` sizes the runtime filter, `N` the ring, and `MSG` each
/// rendered message. All storage is inline; no operation allocates or
/// blocks.
pub struct BufferedLogger<C: Clock, const COMPONENTS: usize, const N: usize, const MSG: usize> {
    clock: C,
    filter: ComponentFilter<COMPONENTS>,
    policy: OverflowPolicy,
    entries: [Entry<MSG>; N],
    head: usize,
    stored: usize,
    dropped: u32,
}

impl<C: Clock, const COMPONENTS: usize, const N: usize, const MSG: usize>
    BufferedLogger<C, COMPONENTS, N, MSG>
{
    /// Create a logger stamping records with `clock`.
    pub const fn new(clock: C, default_level: LevelFilter, policy: OverflowPolicy) -> Self {
        Self {
            clock,
            filter: ComponentFilter::new(default_level),
            policy,
            entries: [Entry::EMPTY; N],
            head: 0,
            stored: 0,
            dropped: 0,
        }
    }

    /// Runtime filter access.
    pub const fn filter(&self) -> &ComponentFilter<COMPONENTS> {
        &self.filter
    }

    /// Mutable runtime filter access.
    pub fn filter_mut(&mut self) -> &mut ComponentFilter<COMPONENTS> {
        &mut self.filter
    }

    /// Number of buffered records.
    pub const fn len(&self) -> usize {
        self.stored
    }

    /// Whether no records are buffered.
    pub const fn is_empty(&self) -> bool {
        self.stored == 0
    }

    /// Records discarded because the ring was full.
    pub const fn dropped(&self) -> u32 {
        self.dropped
    }

    /// Discard all buffered records without touching the drop counter.
    pub fn clear(&mut self) {
        self.stored = 0;
    }

    /// Drain up to `budget` records into `sink`, oldest first.
    ///
    /// The budget bounds the work done per call so a slow sink can be fed
    /// from a low-priority context without starving it.
    pub fn drain_with_budget(&mut self, sink: &mut dyn Sink, budget: usize) -> usize {
        let mut delivered = 0;
        while delivered < budget && self.stored > 0 {
            let entry = self.entries[self.head];
            self.head = (self.head + 1) % N;
            self.stored -= 1;
            sink.consume(
                &Record::new(
                    entry.component,
                    entry.level,
                    entry.timestamp,
                    format_args!("{}", entry.text()),
                ),
                entry.truncated,
            );
            delivered += 1;
        }
        delivered
    }

    /// Drain every buffered record into `sink`, oldest first.
    pub fn drain(&mut self, sink: &mut dyn Sink) -> usize {
        self.drain_with_budget(sink, usize::MAX)
    }

    fn store(&mut self, component: ComponentId, level: Level, args: core::fmt::Arguments<'_>) {
        if N == 0 {
            self.dropped = self.dropped.saturating_add(1);
            return;
        }
        if self.stored == N {
            match self.policy {
                OverflowPolicy::DropNewest => {
                    self.dropped = self.dropped.saturating_add(1);
                    return;
                }
                OverflowPolicy::DropOldest => {
                    self.head = (self.head + 1) % N;
                    self.stored -= 1;
                    self.dropped = self.dropped.saturating_add(1);
                }
            }
        }
        let slot = (self.head + self.stored) % N;
        let entry = &mut self.entries[slot];
        entry.component = component;
        entry.level = level;
        entry.timestamp = self.clock.now();
        let mut writer = EntryWriter {
            bytes: &mut entry.bytes,
            len: 0,
            truncated: false,
        };
        // A formatting failure cannot happen: EntryWriter never errors.
        let _ = core::fmt::write(&mut writer, args);
        entry.len = writer.len;
        entry.truncated = writer.truncated;
        self.stored += 1;
    }
}

impl<C: Clock, const COMPONENTS: usize, const N: usize, const MSG: usize> Log
    for BufferedLogger<C, COMPONENTS, N, MSG>
{
    fn enabled(&self, component: ComponentId, level: Level) -> bool {
        self.filter.enabled(component, level)
    }

    fn log(&mut self, component: ComponentId, level: Level, args: core::fmt::Arguments<'_>) {
        if self.enabled(component, level) {
            self.store(component, level, args);
        }
    }
}

/// Sink rendering `[timestamp-us] LEVEL name: message` lines onto any
/// [`core::fmt::Write`] destination, including no_std UART writers.
pub struct FmtSink<W: core::fmt::Write> {
    writer: W,
    names: &'static [&'static str],
}

impl<W: core::fmt::Write> FmtSink<W> {
    /// Create a sink labeling components from `names` by index.
    pub const fn new(writer: W, names: &'static [&'static str]) -> Self {
        Self { writer, names }
    }

    /// Access the wrapped writer.
    pub const fn writer(&self) -> &W {
        &self.writer
    }

    /// Consume the sink, returning the wrapped writer.
    pub fn into_writer(self) -> W {
        self.writer
    }
}

impl<W: core::fmt::Write> Sink for FmtSink<W> {
    fn consume(&mut self, record: &Record<'_>, truncated: bool) {
        let name = self
            .names
            .get(usize::from(record.component.get()))
            .copied()
            .unwrap_or("?");
        let marker = if truncated { "[..]" } else { "" };
        // Sinks are best-effort by contract; a failing writer must not
        // propagate a panic into the drain path.
        let _ = writeln!(
            self.writer,
            "[{:>10}us] {:8} {}: {}{}",
            record.timestamp.as_nanos() / 1_000,
            record.level.name(),
            name,
            record.args,
            marker,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bsw_log;
    use bsw_time::{Clock, Duration, FakeClock, Instant};
    use core::cell::Cell;

    /// Shared-interior clock so tests can advance time they handed out.
    struct CellClock(Cell<u64>);

    impl Clock for CellClock {
        fn now(&self) -> Instant {
            Instant::from_nanos(self.0.get())
        }
    }

    struct CollectSink {
        lines: Vec<(u8, Level, u64, String, bool)>,
    }

    impl Sink for CollectSink {
        fn consume(&mut self, record: &Record<'_>, truncated: bool) {
            self.lines.push((
                record.component.get(),
                record.level,
                record.timestamp.as_nanos(),
                record.args.to_string(),
                truncated,
            ));
        }
    }

    fn collect() -> CollectSink {
        CollectSink { lines: Vec::new() }
    }

    #[test]
    fn records_are_stamped_and_drained_in_order() {
        let clock = CellClock(Cell::new(100));
        let mut logger: BufferedLogger<&CellClock, 2, 4, 32> =
            BufferedLogger::new(&clock, LevelFilter::Debug, OverflowPolicy::DropNewest);
        bsw_log!(&mut logger, ComponentId::new(0), Level::Info, "first");
        clock.0.set(200);
        bsw_log!(
            &mut logger,
            ComponentId::new(1),
            Level::Error,
            "second {}",
            2
        );
        let mut sink = collect();
        assert_eq!(logger.drain(&mut sink), 2);
        assert_eq!(
            sink.lines,
            vec![
                (0, Level::Info, 100, "first".to_string(), false),
                (1, Level::Error, 200, "second 2".to_string(), false),
            ]
        );
        assert!(logger.is_empty());
    }

    #[test]
    fn missing_sink_never_blocks_and_counts_drops() {
        let mut logger: BufferedLogger<FakeClock, 1, 3, 16> = BufferedLogger::new(
            FakeClock::default(),
            LevelFilter::Debug,
            OverflowPolicy::DropNewest,
        );
        for index in 0..10 {
            bsw_log!(&mut logger, ComponentId::new(0), Level::Info, "m{index}");
        }
        assert_eq!(logger.len(), 3);
        assert_eq!(logger.dropped(), 7);
        let mut sink = collect();
        logger.drain(&mut sink);
        // DropNewest preserves the oldest history.
        assert_eq!(sink.lines[0].3, "m0");
        assert_eq!(sink.lines[2].3, "m2");
    }

    #[test]
    fn drop_oldest_keeps_newest_records() {
        let mut logger: BufferedLogger<FakeClock, 1, 3, 16> = BufferedLogger::new(
            FakeClock::default(),
            LevelFilter::Debug,
            OverflowPolicy::DropOldest,
        );
        for index in 0..5 {
            bsw_log!(&mut logger, ComponentId::new(0), Level::Info, "m{index}");
        }
        assert_eq!(logger.dropped(), 2);
        let mut sink = collect();
        logger.drain(&mut sink);
        let texts: Vec<&str> = sink.lines.iter().map(|line| line.3.as_str()).collect();
        assert_eq!(texts, ["m2", "m3", "m4"]);
    }

    #[test]
    fn long_messages_truncate_at_character_boundaries() {
        let mut logger: BufferedLogger<FakeClock, 1, 2, 8> = BufferedLogger::new(
            FakeClock::default(),
            LevelFilter::Debug,
            OverflowPolicy::DropNewest,
        );
        // "aaaaaaä" is 6 ASCII bytes plus a 2-byte character: 8 bytes fits.
        // "aaaaaaaä" would need 9 bytes: the two-byte character must be cut
        // as a whole, never split.
        bsw_log!(&mut logger, ComponentId::new(0), Level::Info, "aaaaaaä");
        bsw_log!(&mut logger, ComponentId::new(0), Level::Info, "aaaaaaaä");
        let mut sink = collect();
        logger.drain(&mut sink);
        assert_eq!(sink.lines[0].3, "aaaaaaä");
        assert!(!sink.lines[0].4);
        assert_eq!(sink.lines[1].3, "aaaaaaa");
        assert!(sink.lines[1].4);
    }

    #[test]
    fn runtime_filter_blocks_storage() {
        let mut logger: BufferedLogger<FakeClock, 2, 4, 16> = BufferedLogger::new(
            FakeClock::default(),
            LevelFilter::Warn,
            OverflowPolicy::DropNewest,
        );
        bsw_log!(&mut logger, ComponentId::new(0), Level::Info, "hidden");
        bsw_log!(&mut logger, ComponentId::new(0), Level::Warn, "shown");
        assert_eq!(logger.len(), 1);
        logger
            .filter_mut()
            .set(ComponentId::new(0), LevelFilter::Off);
        bsw_log!(&mut logger, ComponentId::new(0), Level::Critical, "hidden");
        assert_eq!(logger.len(), 1);
    }

    #[test]
    fn budgeted_drain_bounds_work_per_call() {
        let mut logger: BufferedLogger<FakeClock, 1, 8, 16> = BufferedLogger::new(
            FakeClock::default(),
            LevelFilter::Debug,
            OverflowPolicy::DropNewest,
        );
        for index in 0..5 {
            bsw_log!(&mut logger, ComponentId::new(0), Level::Info, "m{index}");
        }
        let mut sink = collect();
        assert_eq!(logger.drain_with_budget(&mut sink, 2), 2);
        assert_eq!(logger.len(), 3);
        assert_eq!(logger.drain_with_budget(&mut sink, 10), 3);
        assert!(logger.is_empty());
    }

    #[test]
    fn fmt_sink_renders_labelled_lines() {
        let mut clock = FakeClock::default();
        clock.advance(Duration::from_micros(1500).unwrap());
        let mut logger: BufferedLogger<FakeClock, 2, 4, 32> =
            BufferedLogger::new(clock, LevelFilter::Debug, OverflowPolicy::DropNewest);
        bsw_log!(&mut logger, ComponentId::new(1), Level::Warn, "queue {}", 9);
        let mut sink = FmtSink::new(String::new(), &["lifecycle", "can"]);
        logger.drain(&mut sink);
        let line = sink.into_writer();
        assert_eq!(line, "[      1500us] WARN     can: queue 9\n");
    }

    #[test]
    fn zero_capacity_ring_only_counts() {
        let mut logger: BufferedLogger<FakeClock, 1, 0, 8> = BufferedLogger::new(
            FakeClock::default(),
            LevelFilter::Debug,
            OverflowPolicy::DropOldest,
        );
        bsw_log!(&mut logger, ComponentId::new(0), Level::Info, "x");
        assert_eq!(logger.len(), 0);
        assert_eq!(logger.dropped(), 1);
    }
}
