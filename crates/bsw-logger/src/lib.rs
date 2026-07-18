//! Allocation-free logging front end for the OpenBSW Rust port.
//!
//! This crate mirrors the upstream `util::logger` model: components log
//! through levels that are filtered twice — once at compile time and once at
//! run time — before a record reaches a backend.
//!
//! - [`Level`] and [`LevelFilter`] order severities exactly like upstream
//!   (`CRITICAL` is the most severe, `DEBUG` the least).
//! - [`ComponentId`] and [`ComponentFilter`] give every component its own
//!   runtime threshold.
//! - [`Record`] is the allocation-free unit handed to backends: component,
//!   level, timestamp, and borrowed [`core::fmt::Arguments`].
//! - [`bsw_log!`] evaluates its format arguments only when both filters
//!   pass. The compile-time maximum is an application-owned constant, so a
//!   statement above it folds to nothing — including argument evaluation.
//!
//! Compile-time filtering deliberately uses a macro parameter instead of
//! cargo features: features are additive across a workspace build, which
//! would let one crate's restriction silence another crate's logs.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod buffer;

use bsw_time::Instant;

/// Severity of one log statement. Lower numeric value is more severe.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Level {
    /// Unrecoverable condition; upstream `CRITICAL`.
    Critical = 1,
    /// Operation failed; upstream `ERROR`.
    Error = 2,
    /// Unexpected but tolerated condition; upstream `WARN`.
    Warn = 3,
    /// Normal operational report; upstream `INFO`.
    Info = 4,
    /// Development detail; upstream `DEBUG`.
    Debug = 5,
}

impl Level {
    /// Upstream-style uppercase name.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Critical => "CRITICAL",
            Self::Error => "ERROR",
            Self::Warn => "WARN",
            Self::Info => "INFO",
            Self::Debug => "DEBUG",
        }
    }
}

/// Threshold for a component or for a compile-time maximum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum LevelFilter {
    /// Disable every statement.
    Off = 0,
    /// Allow only `Critical`.
    Critical = 1,
    /// Allow `Critical` and `Error`.
    Error = 2,
    /// Allow up to `Warn`.
    Warn = 3,
    /// Allow up to `Info`.
    Info = 4,
    /// Allow everything.
    Debug = 5,
}

impl LevelFilter {
    /// Whether a statement at `level` passes this threshold.
    pub const fn allows(self, level: Level) -> bool {
        (level as u8) <= (self as u8)
    }
}

/// Index of a component in the application's component table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ComponentId(u8);

impl ComponentId {
    /// Construct a component identifier.
    pub const fn new(raw: u8) -> Self {
        Self(raw)
    }

    /// Numeric component-table index.
    pub const fn get(self) -> u8 {
        self.0
    }
}

/// Per-component runtime level thresholds.
///
/// Components outside the configured range are disabled, so a stale ID can
/// never bypass filtering.
#[derive(Debug, Clone)]
pub struct ComponentFilter<const N: usize> {
    levels: [LevelFilter; N],
}

impl<const N: usize> ComponentFilter<N> {
    /// Create a filter with the same threshold for every component.
    pub const fn new(default: LevelFilter) -> Self {
        Self {
            levels: [default; N],
        }
    }

    /// Set one component's threshold. Returns `false` for unknown IDs.
    pub fn set(&mut self, component: ComponentId, level: LevelFilter) -> bool {
        if let Some(slot) = self.levels.get_mut(usize::from(component.get())) {
            *slot = level;
            true
        } else {
            false
        }
    }

    /// Set every component's threshold.
    pub fn set_all(&mut self, level: LevelFilter) {
        self.levels = [level; N];
    }

    /// Current threshold; unknown IDs report [`LevelFilter::Off`].
    pub fn get(&self, component: ComponentId) -> LevelFilter {
        self.levels
            .get(usize::from(component.get()))
            .copied()
            .unwrap_or(LevelFilter::Off)
    }

    /// Whether a statement passes this component's threshold.
    pub fn enabled(&self, component: ComponentId, level: Level) -> bool {
        self.get(component).allows(level)
    }
}

/// One log statement handed to a backend. Borrows its format arguments, so
/// it never allocates and never outlives the statement.
#[derive(Clone, Copy)]
pub struct Record<'a> {
    /// Originating component.
    pub component: ComponentId,
    /// Statement severity.
    pub level: Level,
    /// Instant stamped by the backend's clock at acceptance time.
    pub timestamp: Instant,
    /// Borrowed message payload.
    pub args: core::fmt::Arguments<'a>,
}

impl<'a> Record<'a> {
    /// Assemble a record.
    pub const fn new(
        component: ComponentId,
        level: Level,
        timestamp: Instant,
        args: core::fmt::Arguments<'a>,
    ) -> Self {
        Self {
            component,
            level,
            timestamp,
            args,
        }
    }
}

/// Logging backend contract used by [`bsw_log!`].
pub trait Log {
    /// Runtime filter decision. Called before format arguments exist, so a
    /// disabled statement never evaluates them.
    fn enabled(&self, component: ComponentId, level: Level) -> bool;

    /// Accept an enabled statement. The backend stamps the timestamp.
    fn log(&mut self, component: ComponentId, level: Level, args: core::fmt::Arguments<'_>);
}

/// Emit one log statement through both filters.
///
/// The first form takes the application's compile-time maximum as a constant
/// expression; statements above it compile to nothing and never evaluate
/// their arguments. The second form applies no compile-time restriction.
///
/// ```
/// use bsw_logger::{bsw_log, ComponentId, Level, LevelFilter, Log};
///
/// struct Null;
/// impl Log for Null {
///     fn enabled(&self, _: ComponentId, _: Level) -> bool {
///         true
///     }
///     fn log(&mut self, _: ComponentId, _: Level, _: core::fmt::Arguments<'_>) {}
/// }
///
/// const MAX: LevelFilter = LevelFilter::Info;
/// let mut logger = Null;
/// let lost_frames = 3;
/// bsw_log!(max: MAX, &mut logger, ComponentId::new(0), Level::Warn, "lost {lost_frames} frames");
/// bsw_log!(&mut logger, ComponentId::new(0), Level::Debug, "unfiltered build");
/// ```
#[macro_export]
macro_rules! bsw_log {
    (max: $max:expr, $logger:expr, $component:expr, $level:expr, $($arg:tt)+) => {{
        const BSW_LOG_STATIC_MAX: $crate::LevelFilter = $max;
        if BSW_LOG_STATIC_MAX.allows($level) {
            let logger = $logger;
            if $crate::Log::enabled(&*logger, $component, $level) {
                $crate::Log::log(logger, $component, $level, ::core::format_args!($($arg)+));
            }
        }
    }};
    ($logger:expr, $component:expr, $level:expr, $($arg:tt)+) => {
        $crate::bsw_log!(max: $crate::LevelFilter::Debug, $logger, $component, $level, $($arg)+)
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::Cell;

    struct Capture {
        filter: ComponentFilter<2>,
        calls: Vec<(u8, Level, String)>,
    }

    impl Capture {
        fn new(default: LevelFilter) -> Self {
            Self {
                filter: ComponentFilter::new(default),
                calls: Vec::new(),
            }
        }
    }

    impl Log for Capture {
        fn enabled(&self, component: ComponentId, level: Level) -> bool {
            self.filter.enabled(component, level)
        }

        fn log(&mut self, component: ComponentId, level: Level, args: core::fmt::Arguments<'_>) {
            self.calls.push((component.get(), level, args.to_string()));
        }
    }

    fn observed(counter: &Cell<u32>) -> u32 {
        counter.set(counter.get() + 1);
        counter.get()
    }

    #[test]
    fn severity_ordering_matches_upstream() {
        assert!(LevelFilter::Off < LevelFilter::Critical);
        assert!(Level::Critical < Level::Debug);
        assert!(LevelFilter::Warn.allows(Level::Error));
        assert!(!LevelFilter::Warn.allows(Level::Info));
        assert!(!LevelFilter::Off.allows(Level::Critical));
        assert_eq!(Level::Critical.name(), "CRITICAL");
    }

    #[test]
    fn component_filter_bounds_and_updates() {
        let mut filter: ComponentFilter<2> = ComponentFilter::new(LevelFilter::Info);
        assert!(filter.enabled(ComponentId::new(0), Level::Info));
        assert!(!filter.enabled(ComponentId::new(0), Level::Debug));
        assert!(filter.set(ComponentId::new(1), LevelFilter::Off));
        assert!(!filter.enabled(ComponentId::new(1), Level::Critical));
        assert!(!filter.set(ComponentId::new(2), LevelFilter::Debug));
        assert_eq!(filter.get(ComponentId::new(9)), LevelFilter::Off);
        filter.set_all(LevelFilter::Debug);
        assert!(filter.enabled(ComponentId::new(1), Level::Debug));
    }

    #[test]
    fn macro_delivers_component_level_and_message() {
        let mut logger = Capture::new(LevelFilter::Debug);
        let value = 7;
        bsw_log!(&mut logger, ComponentId::new(1), Level::Warn, "v={value}");
        assert_eq!(logger.calls, vec![(1, Level::Warn, "v=7".to_string())]);
    }

    #[test]
    fn compile_time_maximum_skips_argument_evaluation() {
        let mut logger = Capture::new(LevelFilter::Debug);
        let evaluations = Cell::new(0);
        bsw_log!(
            max: LevelFilter::Off,
            &mut logger,
            ComponentId::new(0),
            Level::Critical,
            "{}",
            observed(&evaluations)
        );
        bsw_log!(
            max: LevelFilter::Warn,
            &mut logger,
            ComponentId::new(0),
            Level::Info,
            "{}",
            observed(&evaluations)
        );
        assert_eq!(evaluations.get(), 0);
        assert!(logger.calls.is_empty());
    }

    #[test]
    fn runtime_filter_skips_argument_evaluation() {
        let mut logger = Capture::new(LevelFilter::Error);
        let evaluations = Cell::new(0);
        bsw_log!(
            &mut logger,
            ComponentId::new(0),
            Level::Info,
            "{}",
            observed(&evaluations)
        );
        assert_eq!(evaluations.get(), 0);
        assert!(logger.calls.is_empty());
        bsw_log!(
            &mut logger,
            ComponentId::new(0),
            Level::Error,
            "{}",
            observed(&evaluations)
        );
        assert_eq!(evaluations.get(), 1);
        assert_eq!(logger.calls.len(), 1);
    }

    #[test]
    fn record_preserves_all_fields() {
        let record = Record::new(
            ComponentId::new(3),
            Level::Error,
            Instant::from_nanos(99),
            format_args!("x"),
        );
        assert_eq!(record.component.get(), 3);
        assert_eq!(record.level, Level::Error);
        assert_eq!(record.timestamp, Instant::from_nanos(99));
    }
}
