// Copyright 2026 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! Heap-free functional-safety support mechanisms.
//!
//! These mechanisms are engineering aids and are **not** a certified safety
//! library. Integrators remain responsible for the safety concept, hardware
//! fault assumptions, diagnostic coverage analysis, and independent review.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod mechanisms;
pub mod monitor;
pub mod supervisor;
pub mod task;

pub use mechanisms::{
    EccCapabilities, FastTestState, MpuConfig, MpuError, MpuRegion, MpuRegionKind,
    RetainedSafetyEvent, RetainedWatchdogFastTest, RomCheckStatus, RomCrcChecker, WatchdogFastTest,
};
pub use monitor::{
    EventHandler, RegisterEntry, RegisterMonitor, SequenceMonitor, Trigger, ValueMonitor,
    WatchdogMonitor,
};
pub use supervisor::{
    FailureAction, SafeSupervisor, SafetyEvent, SafetyEventCode, SafetyPolicy, SafetySeverity,
    SafetySink,
};
pub use task::{FaultInjection, SafetyTask};
