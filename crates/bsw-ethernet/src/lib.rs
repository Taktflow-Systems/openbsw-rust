//! # bsw-ethernet
//!
//! Ethernet/TCP/UDP abstraction for embedded BSW — Rust port of `OpenBSW` `cpp2ethernet`.
//!
//! Provides IP address types, TCP/UDP socket traits, and network configuration.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod endpoint;
pub mod ip;
pub mod network_config;
pub mod tcp;
pub mod udp;

pub use endpoint::IpEndpoint;
pub use ip::{AddressFamily, IpAddress};
pub use network_config::NetworkConfig;
pub use tcp::{
    ConnectionCloseReason, SendResult, TcpDataListener, TcpError, TcpSendListener,
    TcpServerSocket, TcpSocket,
};
pub use udp::{DatagramPacket, UdpDataListener, UdpError, UdpSendListener, UdpSocket};
