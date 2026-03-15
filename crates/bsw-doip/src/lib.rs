//! # bsw-doip
//!
//! `DoIP` (ISO 13400) diagnostics over IP — Rust port of `OpenBSW` `doip`.
//!
//! Provides `DoIP` header parsing, protocol constants, send job types, and
//! transport parameters.  All types are `no_std`-compatible when the `std`
//! feature is disabled.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod constants;
pub mod header;
pub mod parameters;
pub mod result;
pub mod routing;

pub use constants::{
    ActivationType, DiagNackCode, DiagnosticPowerMode, NackCode, PayloadType, ProtocolVersion,
    RoutingActivationCode, EID_LENGTH, GID_LENGTH, TCP_DATA_PORT, TCP_TLS_PORT,
    UDP_DISCOVERY_PORT, VIN_LENGTH,
};
pub use header::{DoIpHeader, HeaderError, HEADER_SIZE};
pub use parameters::TransportParameters;
pub use result::DoIpResult;
pub use routing::{
    DiagnosticMessageHeader, ParseError, RoutingActivationRequest, RoutingActivationResponse,
    VehicleAnnouncement,
};
