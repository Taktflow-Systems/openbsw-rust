//! Release resource and timing contract shared by production composition and CI.

/// Software receive-queue entries configured for the F413 bxCAN adapter.
pub const F413_CAN_RX_FRAMES: usize = 32;
/// Software receive-queue entries configured for the G474 FDCAN adapter.
pub const G474_CAN_RX_FRAMES: usize = 16;
/// Bounded UART staging capacity used by both production applications.
pub const CONSOLE_TX_BYTES: usize = 512;
/// Maximum ISO-TP diagnostic request payload accepted by the board adapter.
pub const DIAGNOSTIC_PAYLOAD_BYTES: usize = 256;
/// Maximum diagnostic response produced by the shared reference core.
pub const DIAGNOSTIC_RESPONSE_BYTES: usize = 128;

/// Maximum interval between production main-loop scheduling checkpoints.
pub const REFERENCE_CYCLE_US: u64 = 1_000;
/// Period of the externally observable application heartbeat.
pub const HEARTBEAT_PERIOD_US: u64 = 1_000_000;
/// Configured controller recovery delay after bus-off.
pub const CAN_BUS_OFF_RECOVERY_MS: u64 = 1_000;
/// Application-level UDS response deadline used by HIL acceptance.
pub const UDS_RESPONSE_MS: u64 = 100;
/// Hardware timeout used by the single-attempt startup watchdog proof.
pub const WATCHDOG_FAST_TEST_RESET_MS: u64 = 1_000;
/// Fail-safe deadline if the hardware watchdog does not reset the MCU.
pub const WATCHDOG_FAST_TEST_DEADLINE_MS: u64 = 1_500;
/// Mandatory bounded-soak duration for release acceptance.
pub const BOUNDED_SOAK_MINUTES: u64 = 10;
