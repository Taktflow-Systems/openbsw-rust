//! Real POSIX composition of lifecycle, console, I/O, diagnostics, and storage.

pub mod can_demo;
pub mod config;
pub mod diagnostics;
pub mod io;
pub mod lifecycle;
pub mod network;
pub mod scenario;
pub mod storage;

use bsw_console::tokenize;
use bsw_logger::LevelFilter;
use bsw_time::Instant;
use std::fmt::Write as _;

use bsw_reference_core::ProductionComposition;
pub use config::{AppConfig, ConfigError};
use diagnostics::{DiagnosticCore, DiagnosticTransport};
use lifecycle::LifecycleSystem;
use storage::PersistentServices;

/// Fully composed host application. Configuration is supplied, never embedded
/// in component logic.
pub struct ReferenceApp {
    config: AppConfig,
    lifecycle: LifecycleSystem,
    core: ProductionComposition,
    storage: PersistentServices,
    command_count: u64,
}

impl ReferenceApp {
    pub fn new(config: AppConfig) -> Result<Self, ConfigError> {
        config.validate()?;
        Ok(Self {
            config,
            lifecycle: LifecycleSystem::new(),
            core: ProductionComposition::new(),
            storage: PersistentServices::new().expect("static storage geometry is valid"),
            command_count: 0,
        })
    }

    pub fn start(&mut self, now: Instant) -> Result<(), &'static str> {
        if let Ok(Some(counter)) = self
            .storage
            .read_counter(self.config.storage.diagnostic_block)
        {
            self.core.diagnostics_mut().restore_counter(counter);
        }
        self.core.start(now);
        self.lifecycle.start(now)
    }

    pub fn shutdown(&mut self, now: Instant) -> Result<(), &'static str> {
        self.storage
            .write_counter(
                self.config.storage.diagnostic_block,
                self.core.diagnostics().persistent_counter(),
            )
            .map_err(|_| "storage write failed")?;
        self.core.shutdown(now);
        self.lifecycle.shutdown(now)
    }

    pub const fn level(&self) -> u8 {
        self.lifecycle.level()
    }

    pub fn logs(&self) -> &[String] {
        self.lifecycle.logs()
    }

    pub fn diagnostics_mut(&mut self) -> &mut DiagnosticCore {
        self.core.diagnostics_mut()
    }

    #[allow(clippy::too_many_lines)] // command tree is intentionally visible in one dispatch table
    pub fn command(&mut self, line: &str, now: Instant) -> String {
        self.command_count = self.command_count.saturating_add(1);
        let Ok(tokens) = tokenize(line) else {
            return "error: too many arguments".into();
        };
        let args = tokens.as_slice();
        match args {
            ["help"] => concat!(
                "can info|send <id> <hex>; lc level <0-9>|reboot|poweroff; ",
                "stats all; logger level <debug|info|warn|error>; ",
                "diag <hex>; io adc <0-4095>|gpio <0|1>; storage get|set <u32>; trace status"
            )
            .into(),
            ["lc", "level", level] => match level.parse::<u8>() {
                Ok(level) if self.lifecycle.transition_to(level, now).is_ok() => {
                    format!("level {level}")
                }
                _ => "error: level".into(),
            },
            ["lc", "reboot"] => {
                if self.lifecycle.shutdown(now).is_ok() && self.lifecycle.start(now).is_ok() {
                    "rebooted".into()
                } else {
                    "error: reboot".into()
                }
            }
            ["lc", "poweroff"] => match self.shutdown(now) {
                Ok(()) => "powered off".into(),
                Err(error) => format!("error: {error}"),
            },
            ["logger", "level", level] => {
                let parsed = match *level {
                    "debug" => Some(LevelFilter::Debug),
                    "info" => Some(LevelFilter::Info),
                    "warn" => Some(LevelFilter::Warn),
                    "error" => Some(LevelFilter::Error),
                    _ => None,
                };
                if let Some(level) = parsed {
                    self.lifecycle.set_log_level(level);
                    format!("logger {}", level_name(level))
                } else {
                    "error: logger level".into()
                }
            }
            ["stats", "all"] => format!(
                "commands={} diagnostics={}",
                self.command_count,
                self.core.diagnostics().dispatch_count()
            ),
            ["diag", request] => match decode_hex(request) {
                Ok(bytes) => encode_hex(
                    self.core
                        .diagnostics_mut()
                        .dispatch_at(DiagnosticTransport::Console, &bytes, now)
                        .bytes(),
                ),
                Err(()) => "error: hex".into(),
            },
            ["io", "adc", value] => match value
                .parse::<u16>()
                .ok()
                .and_then(|value| self.core.cycle_io(value, now).ok())
            {
                Some(snapshot) => format!(
                    "adc={} speed={} pwm={}",
                    snapshot.adc, snapshot.vehicle_speed, snapshot.pwm_permille
                ),
                None => "error: adc".into(),
            },
            ["io", "gpio", value] if matches!(*value, "0" | "1") => {
                self.core.set_output(*value == "1", now);
                format!("gpio={}", value)
            }
            ["storage", "get"] => match self.storage.read_counter(self.config.storage.demo_block) {
                Ok(Some(value)) => value.to_string(),
                Ok(None) => "unset".into(),
                Err(_) => "error: storage".into(),
            },
            ["storage", "set", value] => match value.parse::<u32>() {
                Ok(value)
                    if self
                        .storage
                        .write_counter(self.config.storage.demo_block, value)
                        .is_ok() =>
                {
                    "stored".into()
                }
                _ => "error: storage".into(),
            },
            ["can", "info"] => format!(
                "request={:03x} response={:03x}",
                self.config.diagnostics.can_request_id, self.config.diagnostics.can_response_id
            ),
            ["can", "send", id, data] => {
                if u32::from_str_radix(id.trim_start_matches("0x"), 16).is_ok()
                    && decode_hex(data).is_ok()
                {
                    "sent".into()
                } else {
                    "error: can".into()
                }
            }
            ["trace", "status"] => {
                if self.config.features.tracing {
                    "tracing enabled".into()
                } else {
                    "tracing disabled".into()
                }
            }
            _ => "error: unknown command".into(),
        }
    }
}

fn level_name(level: LevelFilter) -> &'static str {
    match level {
        LevelFilter::Off => "OFF",
        LevelFilter::Critical => "CRITICAL",
        LevelFilter::Error => "ERROR",
        LevelFilter::Warn => "WARN",
        LevelFilter::Info => "INFO",
        LevelFilter::Debug => "DEBUG",
    }
}

fn decode_hex(value: &str) -> Result<Vec<u8>, ()> {
    if !value.len().is_multiple_of(2) {
        return Err(());
    }
    value
        .as_bytes()
        .chunks_exact(2)
        .map(|pair| {
            let text = std::str::from_utf8(pair).map_err(|_| ())?;
            u8::from_str_radix(text, 16).map_err(|_| ())
        })
        .collect()
}

fn encode_hex(bytes: &[u8]) -> String {
    bytes.iter().fold(String::new(), |mut output, byte| {
        let _ = write!(output, "{byte:02x}");
        output
    })
}
