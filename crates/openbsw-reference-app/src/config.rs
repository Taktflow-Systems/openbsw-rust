//! Application-owned capacities, routes, storage blocks, and feature switches.

use std::path::PathBuf;

/// Fixed capacities used by the composed host application.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Capacities {
    pub lifecycle_components: usize,
    pub console_commands: usize,
    pub log_records: usize,
    pub diagnostic_connections: usize,
    pub transport_payload: usize,
    pub storage_blocks: usize,
}

/// Diagnostic addressing and periodic transport configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiagnosticConfig {
    pub can_request_id: u32,
    pub can_response_id: u32,
    pub tester_address: u16,
    pub entity_address: u16,
    pub cycle_us: u32,
}

/// Host network endpoints. Zero ports request ephemeral test ports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NetworkConfig {
    pub doip_port: u16,
    pub udp_echo_port: u16,
    pub tcp_echo_port: u16,
}

/// Application-visible persistent block assignments.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StorageConfig {
    pub demo_block: u16,
    pub diagnostic_block: u16,
    pub blob_descriptor: u16,
    pub blob_chunk_base: u16,
    pub region_size: usize,
    pub erase_unit: usize,
    pub program_unit: usize,
}

/// Optional application surfaces; protocol behavior does not depend on them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools)] // independent compile/runtime feature toggles
pub struct FeatureSwitches {
    pub socketcan: bool,
    pub tracing: bool,
    pub simulated_io: bool,
    pub persistence: bool,
}

/// Complete reference-application configuration, kept separate from logic.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AppConfig {
    pub capacities: Capacities,
    pub diagnostics: DiagnosticConfig,
    pub network: NetworkConfig,
    pub storage: StorageConfig,
    pub features: FeatureSwitches,
    pub state_file: PathBuf,
}

/// Configuration defect rejected before any component starts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    ZeroCapacity,
    DuplicateCanId,
    DuplicateLogicalAddress,
    InvalidStorageGeometry,
    DuplicateStorageBlock,
}

impl AppConfig {
    /// Validate cross-field invariants without touching the platform.
    pub fn validate(&self) -> Result<(), ConfigError> {
        let c = self.capacities;
        if [
            c.lifecycle_components,
            c.console_commands,
            c.log_records,
            c.diagnostic_connections,
            c.transport_payload,
            c.storage_blocks,
        ]
        .contains(&0)
        {
            return Err(ConfigError::ZeroCapacity);
        }
        if self.diagnostics.can_request_id == self.diagnostics.can_response_id {
            return Err(ConfigError::DuplicateCanId);
        }
        if self.diagnostics.tester_address == self.diagnostics.entity_address {
            return Err(ConfigError::DuplicateLogicalAddress);
        }
        let storage = self.storage;
        if storage.region_size == 0
            || storage.erase_unit == 0
            || storage.program_unit == 0
            || !storage.region_size.is_multiple_of(storage.erase_unit)
            || !storage.erase_unit.is_multiple_of(storage.program_unit)
        {
            return Err(ConfigError::InvalidStorageGeometry);
        }
        let blocks = [
            storage.demo_block,
            storage.diagnostic_block,
            storage.blob_descriptor,
            storage.blob_chunk_base,
        ];
        for (index, block) in blocks.iter().enumerate() {
            if blocks[index + 1..].contains(block) {
                return Err(ConfigError::DuplicateStorageBlock);
            }
        }
        Ok(())
    }
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            capacities: Capacities {
                lifecycle_components: 16,
                console_commands: 16,
                log_records: 64,
                diagnostic_connections: 5,
                transport_payload: 256,
                storage_blocks: 16,
            },
            diagnostics: DiagnosticConfig {
                can_request_id: 0x02a,
                can_response_id: 0x0f0,
                tester_address: 0x0e80,
                entity_address: 0x0e00,
                cycle_us: 10_000,
            },
            network: NetworkConfig {
                doip_port: 13_400,
                udp_echo_port: 49_444,
                tcp_echo_port: 49_555,
            },
            storage: StorageConfig {
                demo_block: 0x0a01,
                diagnostic_block: 0x0a02,
                blob_descriptor: 0x0a10,
                blob_chunk_base: 0x0a20,
                region_size: 16_384,
                erase_unit: 1_024,
                program_unit: 4,
            },
            features: FeatureSwitches {
                socketcan: cfg!(feature = "socketcan"),
                tracing: cfg!(feature = "tracing"),
                simulated_io: true,
                persistence: true,
            },
            state_file: PathBuf::from("target/reference-app/state.bin"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_configuration_is_complete_and_valid() {
        let config = AppConfig::default();
        assert_eq!(config.diagnostics.can_request_id, 0x02a);
        assert_eq!(config.storage.demo_block, 0x0a01);
        assert_eq!(config.validate(), Ok(()));
    }

    #[test]
    fn invalid_cross_field_configuration_is_rejected() {
        let mut config = AppConfig::default();
        config.diagnostics.can_response_id = config.diagnostics.can_request_id;
        assert_eq!(config.validate(), Err(ConfigError::DuplicateCanId));
    }
}
