//! # bsw-doip
//!
//! `DoIP` (ISO 13400) diagnostics over IP — Rust port of `OpenBSW` `doip`.
//!
//! Provides `DoIP` header parsing, protocol constants, send job types, and
//! transport parameters.  All types are `no_std`-compatible when the `std`
//! feature is disabled.

#![cfg_attr(not(feature = "std"), no_std)]

// Unit tests run on the host and may use `std` even in the `no_std`
// configuration of the library.
#[cfg(test)]
extern crate std;

pub mod connection;
pub mod constants;
pub mod diagnostic;
pub mod discovery;
#[cfg(feature = "std")]
pub mod entity;
pub mod header;
pub mod parameters;
pub mod payload;
pub mod result;
pub mod routing;
pub mod server;

pub use connection::{
    Action, ActivationDecision, ActivationPolicy, CloseMode, ConnectionState,
    DefaultActivationPolicy, Frame, MessageHandler, NoMessageHandler, PolicyAction,
    ServerConnection, CONTROL_FRAME_MAX,
};
pub use constants::{
    ActivationType, DiagNackCode, DiagnosticPowerMode, NackCode, PayloadType, ProtocolVersion,
    RoutingActivationCode, EID_LENGTH, GID_LENGTH, TCP_DATA_PORT, TCP_TLS_PORT, UDP_DISCOVERY_PORT,
    VIN_LENGTH,
};
pub use diagnostic::{
    DiagnosticGateway, DiagnosticHandlerAction, DiagnosticMessageHandler,
    DiagnosticMessageListener, DiagnosticSendError, DiagnosticSendToken, DiagnosticSender,
    DiagnosticWireFrame, PoolDiagnosticGateway, ReceivedDiagnosticAck,
};
pub use discovery::{AnnouncementSchedule, DiscoveryEntity};
#[cfg(feature = "std")]
pub use entity::{
    DoIpApplication, EntityAdmissionPolicy, EntityConfig, EntityError, EntityState, PosixDoIpEntity,
};
pub use header::{DoIpHeader, HeaderError, HEADER_SIZE};
pub use parameters::TransportParameters;
pub use payload::{
    CodecError, DiagnosticAck, DiagnosticPayload, EntityStatus, Packet, Payload,
    VehicleIdentification,
};
pub use result::DoIpResult;
pub use routing::{
    DiagnosticMessageHeader, ParseError, RoutingActivationRequest, RoutingActivationResponse,
    VehicleAnnouncement,
};
pub use server::{AcceptError, ConnectionId, ServerEvent, ServerTransportLayer, WireAction};
