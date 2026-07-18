//! Allocation-free service-oriented middleware contracts (packages E05-E08).
//!
//! Runtime protocol types stay `no_std` and heap-free. The `std` feature adds
//! a checked model parser and deterministic Rust generator used at build/CI
//! time; generated code only depends on the runtime contracts in this crate.

#![cfg_attr(not(feature = "std"), no_std)]

extern crate self as bsw_middleware;

#[cfg(feature = "std")]
pub mod codegen;
pub mod generated;
pub mod message;

pub use message::{
    ErrorState, Header, Message, MessageError, MessageKind, MiddlewareResult, MiddlewareTransport,
};

/// Maximum method/member identifier supported by upstream middleware.
pub const MAX_MEMBER_ID: u16 = 128;

/// Primitive payload types understood by the representative generator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrimitiveType {
    /// Empty payload.
    Unit,
    /// Boolean encoded as a Rust `bool`.
    Bool,
    /// Unsigned 8-bit integer.
    U8,
    /// Unsigned 16-bit integer.
    U16,
    /// Unsigned 32-bit integer.
    U32,
    /// Signed 16-bit integer.
    I16,
}

/// Generated method contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MemberDescriptor {
    /// Stable Rust/model name.
    pub name: &'static str,
    /// Member identifier, bounded by [`MAX_MEMBER_ID`].
    pub id: u16,
    /// Request payload type.
    pub request: PrimitiveType,
    /// Response payload type.
    pub response: PrimitiveType,
}

/// Generated service contract.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ServiceDescriptor {
    /// Stable service name.
    pub name: &'static str,
    /// Service identifier.
    pub id: u16,
    /// Service methods.
    pub members: &'static [MemberDescriptor],
}

/// Generated cluster route.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClusterRoute {
    /// Routed service identifier.
    pub service_id: u16,
    /// Source cluster.
    pub source_cluster: u8,
    /// Target cluster.
    pub target_cluster: u8,
}
