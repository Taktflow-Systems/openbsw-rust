//! # bsw-ethernet
//!
//! Ethernet/TCP/UDP abstraction for embedded BSW — Rust port of `OpenBSW` `cpp2ethernet`.
//!
//! Provides IP address types, TCP/UDP socket traits, network configuration,
//! an lwIP-style socket boundary with a deterministic in-memory fake
//! ([`lwip`]), and POSIX adapters over `std::net` ([`posix`], `std` only).

#![cfg_attr(not(feature = "std"), no_std)]

pub mod endpoint;
pub mod ip;
pub mod lwip;
pub mod network_config;
pub mod network_interface;
#[cfg(feature = "std")]
pub mod posix;
pub mod tcp;
pub mod udp;

pub use endpoint::IpEndpoint;
pub use ip::{AddressFamily, IpAddress};
pub use network_config::NetworkConfig;
pub use network_interface::{
    BackendError as NetworkBackendError, ConfigChange, ConfigRegistry, InterfaceBackend,
    InterfaceState, NetworkInterface,
};
pub use tcp::{
    ConnectionCloseReason, SendResult, TcpDataListener, TcpError, TcpSendListener, TcpServerSocket,
    TcpSocket,
};
pub use udp::{DatagramPacket, UdpDataListener, UdpError, UdpSendListener, UdpSocket};
