//! Run-level composition and stable reference-application logging.

use bsw_lifecycle::runlevel::{ErrorPolicy, RunLevelManager, StepStatus};
use bsw_lifecycle::{LifecycleComponent, TransitionResult};
use bsw_logger::{ComponentFilter, ComponentId, Level, LevelFilter};
use bsw_time::Instant;

const COMPONENT_COUNT: usize = 9;
const LEVEL_COUNT: usize = 9;

/// Stable log collector using the common logger level/filter vocabulary.
pub struct RefLogger {
    filter: ComponentFilter<COMPONENT_COUNT>,
    lines: Vec<String>,
}

impl RefLogger {
    pub fn new() -> Self {
        Self {
            filter: ComponentFilter::new(LevelFilter::Debug),
            lines: Vec::new(),
        }
    }

    pub fn set_all(&mut self, level: LevelFilter) {
        self.filter.set_all(level);
    }

    pub fn emit(&mut self, component: usize, level: Level, at: Instant, message: &str) {
        let id = ComponentId::new(component as u8);
        if self.filter.enabled(id, level) {
            self.lines.push(format!(
                "{}: RefApp: {}: {}: {}",
                at.as_nanos() / 1_000_000,
                COMPONENT_NAMES[component],
                level.name(),
                message
            ));
        }
    }

    pub fn lines(&self) -> &[String] {
        &self.lines
    }

    pub fn take(&mut self) -> Vec<String> {
        std::mem::take(&mut self.lines)
    }
}

impl Default for RefLogger {
    fn default() -> Self {
        Self::new()
    }
}

const COMPONENT_NAMES: [&str; COMPONENT_COUNT] = [
    "RUNTIME",
    "SAFETY",
    "CAN",
    "TRANSPORT",
    "STORAGE",
    "DOIP",
    "UDS",
    "SYSADMIN",
    "DEMO",
];
const COMPONENT_LEVELS: [u8; COMPONENT_COUNT] = [1, 1, 2, 4, 5, 6, 7, 8, 9];

struct NamedComponent {
    name: &'static str,
    running: bool,
}

impl LifecycleComponent for NamedComponent {
    fn init(&mut self) -> TransitionResult {
        TransitionResult::Done
    }

    fn run(&mut self) -> TransitionResult {
        self.running = true;
        TransitionResult::Done
    }

    fn shutdown(&mut self) -> TransitionResult {
        self.running = false;
        TransitionResult::Done
    }

    fn name(&self) -> &str {
        self.name
    }
}

/// Upstream-shaped nine-level lifecycle with synchronous host components.
pub struct LifecycleSystem {
    manager: RunLevelManager<COMPONENT_COUNT, LEVEL_COUNT>,
    components: [NamedComponent; COMPONENT_COUNT],
    logger: RefLogger,
}

impl LifecycleSystem {
    pub fn new() -> Self {
        let mut manager = RunLevelManager::new(ErrorPolicy::Rollback);
        for level in COMPONENT_LEVELS {
            manager
                .add_component(level)
                .expect("static lifecycle table fits");
        }
        Self {
            manager,
            components: COMPONENT_NAMES.map(|name| NamedComponent {
                name,
                running: false,
            }),
            logger: RefLogger::new(),
        }
    }

    pub fn transition_to(&mut self, level: u8, now: Instant) -> Result<(), &'static str> {
        self.manager
            .transition_to_level(level)
            .map_err(|_| "invalid run level")?;
        let mut components = self
            .components
            .each_mut()
            .map(|component| component as &mut dyn LifecycleComponent);
        match self.manager.step(&mut components, now) {
            StepStatus::Idle => {
                self.logger.emit(
                    0,
                    Level::Info,
                    now,
                    &format!("run level {} reached", self.manager.current_level()),
                );
                Ok(())
            }
            StepStatus::InProgress => Err("unexpected pending host transition"),
            StepStatus::Faulted => Err("lifecycle transition failed"),
        }
    }

    pub fn start(&mut self, now: Instant) -> Result<(), &'static str> {
        self.logger.emit(0, Level::Info, now, "hello");
        self.transition_to(9, now)
    }

    pub fn shutdown(&mut self, now: Instant) -> Result<(), &'static str> {
        self.transition_to(0, now)?;
        self.logger.emit(0, Level::Info, now, "shutdown complete");
        Ok(())
    }

    pub const fn level(&self) -> u8 {
        self.manager.current_level()
    }

    pub fn set_log_level(&mut self, level: LevelFilter) {
        self.logger.set_all(level);
    }

    pub fn logs(&self) -> &[String] {
        self.logger.lines()
    }

    pub fn take_logs(&mut self) -> Vec<String> {
        self.logger.take()
    }
}

impl Default for LifecycleSystem {
    fn default() -> Self {
        Self::new()
    }
}
