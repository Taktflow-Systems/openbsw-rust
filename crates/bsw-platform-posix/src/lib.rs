//! POSIX host platform aggregation layer for the OpenBSW Rust port
//! (package F01).
//!
//! Protocol and service crates depend on the `bsw-platform` traits and the
//! per-domain contracts (CAN transceiver, sockets, storage backends);
//! *this* crate is what a POSIX reference application composes against. It
//! provides one coherent import surface:
//!
//! - implementations of the six `bsw-platform` contracts for POSIX hosts
//!   ([`PosixResetControl`], [`SoftWatchdog`], [`PosixCriticalSection`],
//!   [`PosixEntropy`], [`PosixUniqueId`], [`PosixPlatformInfo`]),
//! - re-exports of the existing POSIX adapters from the domain crates
//!   ([`clock`], [`executor`], [`console`], [`can`], [`net`],
//!   [`storage`]),
//! - deterministic scenario controls ([`test_control`]) mirroring
//!   [`bsw_platform::mock`] ergonomics.
//!
//! This is a std-only crate by design — the one crate class in the
//! workspace where std is expected. It builds and tests on both Windows
//! and Linux hosts.
//!
//! # Real vs. simulated
//!
//! | Area | Backing | Nature |
//! |------|---------|--------|
//! | Clock | `std::time::Instant` ([`clock::StdClock`]) | real |
//! | Executor | threads + condvar ([`executor::PosixExecutor`]) | real |
//! | Console | stdio ([`console::InputPump`] / [`console::IoWriter`]) | real |
//! | UDP/TCP | `std::net` ([`net`]) | real |
//! | File storage | filesystem ([`storage::FileBackend`]) | real |
//! | SocketCAN | Linux kernel CAN ([`can`], `socketcan` feature) | real (Linux only) |
//! | Reset | recorded request queue ([`PosixResetControl`]) | simulation |
//! | Watchdog | injected-clock software watchdog ([`SoftWatchdog`]) | simulation |
//! | Critical section | process-wide reentrant lock ([`PosixCriticalSection`]) | simulation |
//! | Entropy | seeded PRNG, **not cryptographic** ([`PosixEntropy`]) | simulation |
//! | Unique id | hashed hostname ([`PosixUniqueId`]) | simulation |
//!
//! The reset simulation follows the upstream POSIX application, which maps
//! `softwareSystemReset` to a lifecycle restart rather than a process
//! kill; see [`reset`] for the explicit process-exit escape hatch.
//!
//! # Feature `socketcan` (Linux only)
//!
//! Forwards to `bsw-can/socketcan`. `bsw-can` gates the adapter on
//! `target_os = "linux"`, so enabling the feature elsewhere is inert and
//! still compiles; the Linux `socketcan` CI job validates the real
//! adapter against a provisioned `vcan0`. [`can::VirtualCanBus`] is always
//! available on every host.

pub mod reset;

mod critical;
mod entropy;
mod ident;
pub mod test_control;
mod watchdog;

pub use critical::PosixCriticalSection;
pub use entropy::PosixEntropy;
pub use ident::{PosixPlatformInfo, PosixUniqueId};
pub use reset::{PosixResetControl, ResetKind};
pub use test_control::{FixedEntropy, SharedClock, TestControls};
pub use watchdog::{SoftWatchdog, WatchdogHealth};

// One-import surface for the platform contracts and their mocks.
pub use bsw_platform::{
    mock, with_critical, CriticalSection, Entropy, EntropyError, PlatformInfo, ResetControl,
    ResetReason, UniqueId, Watchdog, WatchdogError, UNIQUE_ID_MAX,
};

/// Monotonic time: contracts plus the host clock and the test clock.
pub mod clock {
    pub use bsw_time::{Clock, Duration, FakeClock, Instant, StdClock};
}

/// Cooperative execution on a host thread: the POSIX executor and the
/// runnable contracts it schedules.
pub mod executor {
    pub use bsw_async::posix::{PosixExecutor, Wakeup};
    pub use bsw_async::{PostError, RunContext, Runnable, TaskId};
}

/// Console front end over stdio.
pub mod console {
    pub use bsw_console::posix::{InputPump, IoWriter, PumpStatus};
}

/// CAN buses for hosts: the in-process [`VirtualCanBus`] everywhere, and
/// the Linux SocketCAN adapter behind the `socketcan` feature.
pub mod can {
    pub use bsw_can::virtual_bus::{VirtualCanBus, VirtualNode};

    #[cfg(all(feature = "socketcan", target_os = "linux"))]
    pub use bsw_can::socketcan::{
        ErrorFrame, KernelFilter, RxEvent, SocketCan, MAX_KERNEL_FILTERS,
    };
}

/// UDP/TCP sockets over `std::net`.
pub mod net {
    pub use bsw_ethernet::posix::{
        from_std_ip, to_std_ip, PosixTcpServerSocket, PosixTcpSocket, PosixUdpSocket,
    };
}

/// Persistent storage backends for hosts: real files, in-memory flash
/// semantics, and deterministic power-cut injection.
pub mod storage {
    pub use bsw_storage::fault::{CutPlan, CutPointBackend};
    pub use bsw_storage::file::{FileBackend, FileGeometry};
    pub use bsw_storage::mem::MemBackend;
    pub use bsw_storage::{StorageBackend, StorageError};
}
