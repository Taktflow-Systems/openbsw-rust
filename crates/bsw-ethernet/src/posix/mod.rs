//! POSIX socket adapters backed by [`std::net`].
//!
//! This module is only available with the `std` feature. It provides
//! concrete, non-blocking implementations of the crate's socket traits on
//! top of the operating system's socket API:
//!
//! * [`PosixUdpSocket`] — implements [`crate::udp::UdpSocket`]
//! * [`PosixTcpSocket`] — implements [`crate::tcp::TcpSocket`]
//! * [`PosixTcpServerSocket`] — implements [`crate::tcp::TcpServerSocket`]
//!
//! All sockets are switched to non-blocking mode after creation so that the
//! polling-style contracts of the traits (upstream `cpp2ethernet` semantics)
//! can be honoured without ever blocking the caller.

pub mod tcp;
pub mod udp;

pub use tcp::{PosixTcpServerSocket, PosixTcpSocket};
pub use udp::PosixUdpSocket;

use std::net::{IpAddr, Ipv4Addr};

use crate::ip::IpAddress;

/// Convert a crate [`IpAddress`] into a [`std::net::IpAddr`].
pub fn to_std_ip(addr: IpAddress) -> IpAddr {
    match addr {
        IpAddress::V4(bytes) => IpAddr::V4(Ipv4Addr::from(bytes)),
        #[cfg(feature = "ipv6")]
        IpAddress::V6(bytes) => IpAddr::V6(std::net::Ipv6Addr::from(bytes)),
    }
}

/// Convert a [`std::net::IpAddr`] into a crate [`IpAddress`].
///
/// Returns `None` for an IPv6 address when the `ipv6` feature is disabled.
pub fn from_std_ip(addr: IpAddr) -> Option<IpAddress> {
    match addr {
        IpAddr::V4(v4) => Some(IpAddress::V4(v4.octets())),
        IpAddr::V6(v6) => {
            #[cfg(feature = "ipv6")]
            {
                Some(IpAddress::V6(v6.octets()))
            }
            #[cfg(not(feature = "ipv6"))]
            {
                let _ = v6;
                None
            }
        }
    }
}

/// Convert an optional [`std::net::SocketAddr`] into a crate [`IpAddress`],
/// falling back to the unspecified address when absent or unrepresentable.
pub(crate) fn addr_or_unspecified(addr: Option<std::net::SocketAddr>) -> IpAddress {
    match addr {
        Some(sa) => from_std_ip(sa.ip()).unwrap_or_default(),
        None => IpAddress::unspecified(),
    }
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ipv4_roundtrip() {
        let addr = IpAddress::ipv4(127, 0, 0, 1);
        let std_addr = to_std_ip(addr);
        assert_eq!(std_addr, IpAddr::V4(Ipv4Addr::LOCALHOST));
        assert_eq!(from_std_ip(std_addr), Some(addr));
    }

    #[test]
    fn addr_or_unspecified_none() {
        assert_eq!(addr_or_unspecified(None), IpAddress::unspecified());
    }

    #[test]
    fn addr_or_unspecified_some() {
        let sa: std::net::SocketAddr = "127.0.0.1:8080".parse().unwrap();
        assert_eq!(addr_or_unspecified(Some(sa)), IpAddress::ipv4(127, 0, 0, 1));
    }

    #[cfg(feature = "ipv6")]
    #[test]
    fn ipv6_roundtrip() {
        let mut bytes = [0u8; 16];
        bytes[15] = 1;
        let addr = IpAddress::ipv6(bytes);
        let std_addr = to_std_ip(addr);
        assert_eq!(from_std_ip(std_addr), Some(addr));
    }

    #[cfg(not(feature = "ipv6"))]
    #[test]
    fn ipv6_unrepresentable_without_feature() {
        let v6: IpAddr = "::1".parse().unwrap();
        assert_eq!(from_std_ip(v6), None);
    }
}
