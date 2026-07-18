//! # bsw-uds
//!
//! UDS diagnostics (ISO 14229) service framework — Rust port of `OpenBSW` `uds`.
//!
//! Provides diagnostic service routing, NRC codes, session management, and service handlers.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod async_jobs;
pub mod connection;
pub mod dem;
pub mod diag_job;
pub mod dispatcher;
pub mod nrc;
pub mod registries;
pub mod service_id;
pub mod services;
pub mod session;
pub mod standard_services;
pub mod state;
pub mod timing;

pub use dem::DemManager;
pub use diag_job::{DiagJob, DiagResult, DiagRouter};
pub use nrc::Nrc;
pub use service_id::ServiceId;
pub use services::{
    ControlDtcSetting, DiagnosticSessionControl, DtcSettingType, EcuReset, ResetType, TesterPresent,
};
pub use session::{DiagSession, SessionMask};
