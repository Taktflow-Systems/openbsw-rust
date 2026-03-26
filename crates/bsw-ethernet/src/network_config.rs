//! Network interface configuration.

use crate::ip::IpAddress;

/// Network interface configuration.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum NetworkConfig {
    /// No valid configuration.
    #[default]
    Invalid,
    /// IPv4 configuration with IP address, netmask, and default gateway.
    Ipv4 {
        /// Host IP address in network byte order.
        ip: [u8; 4],
        /// Subnet mask in network byte order.
        netmask: [u8; 4],
        /// Default gateway in network byte order.
        gateway: [u8; 4],
    },
    /// IPv6 configuration.
    #[cfg(feature = "ipv6")]
    Ipv6 {
        /// IPv6 address in network byte order.
        address: [u8; 16],
    },
}

impl NetworkConfig {
    /// Returns the invalid (unconfigured) variant.
    #[inline]
    pub const fn invalid() -> Self {
        Self::Invalid
    }

    /// Create an IPv4 configuration.
    #[inline]
    pub const fn ipv4(ip: [u8; 4], netmask: [u8; 4], gateway: [u8; 4]) -> Self {
        Self::Ipv4 { ip, netmask, gateway }
    }

    /// Create an IPv6 configuration.
    #[cfg(feature = "ipv6")]
    #[inline]
    pub const fn ipv6(address: [u8; 16]) -> Self {
        Self::Ipv6 { address }
    }

    /// Returns `true` if the configuration is not [`Invalid`](Self::Invalid).
    #[inline]
    pub const fn is_valid(&self) -> bool {
        !matches!(self, Self::Invalid)
    }

    /// Returns the configured IP address, or `None` if invalid.
    pub fn ip_address(&self) -> Option<IpAddress> {
        match self {
            Self::Invalid => None,
            Self::Ipv4 { ip, .. } => Some(IpAddress::ipv4_from_bytes(ip)),
            #[cfg(feature = "ipv6")]
            Self::Ipv6 { address } => Some(IpAddress::ipv6(*address)),
        }
    }

    /// Returns the subnet mask as bytes, or `None` for non-IPv4 / invalid.
    pub const fn netmask(&self) -> Option<[u8; 4]> {
        match self {
            Self::Ipv4 { netmask, .. } => Some(*netmask),
            Self::Invalid => None,
            #[cfg(feature = "ipv6")]
            Self::Ipv6 { .. } => None,
        }
    }

    /// Returns the default gateway as bytes, or `None` for non-IPv4 / invalid.
    pub const fn gateway(&self) -> Option<[u8; 4]> {
        match self {
            Self::Ipv4 { gateway, .. } => Some(*gateway),
            Self::Invalid => None,
            #[cfg(feature = "ipv6")]
            Self::Ipv6 { .. } => None,
        }
    }

    /// Compute the IPv4 broadcast address (`ip | !netmask`).
    ///
    /// Returns `None` for non-IPv4 or invalid configurations.
    pub const fn broadcast(&self) -> Option<[u8; 4]> {
        match self {
            Self::Ipv4 { ip, netmask, .. } => Some([
                ip[0] | !netmask[0],
                ip[1] | !netmask[1],
                ip[2] | !netmask[2],
                ip[3] | !netmask[3],
            ]),
            Self::Invalid => None,
            #[cfg(feature = "ipv6")]
            Self::Ipv6 { .. } => None,
        }
    }
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ip::IpAddress;

    #[test]
    fn invalid_is_not_valid() {
        assert!(!NetworkConfig::invalid().is_valid());
    }

    #[test]
    fn ipv4_config() {
        let cfg = NetworkConfig::ipv4(
            [192, 168, 1, 100],
            [255, 255, 255, 0],
            [192, 168, 1, 1],
        );
        assert!(cfg.is_valid());
    }

    #[test]
    fn ipv4_is_valid() {
        let cfg = NetworkConfig::ipv4([10, 0, 0, 5], [255, 0, 0, 0], [10, 0, 0, 1]);
        assert!(cfg.is_valid());
    }

    #[test]
    fn ip_address_from_ipv4() {
        let cfg = NetworkConfig::ipv4(
            [192, 168, 1, 100],
            [255, 255, 255, 0],
            [192, 168, 1, 1],
        );
        assert_eq!(cfg.ip_address(), Some(IpAddress::ipv4(192, 168, 1, 100)));
    }

    #[test]
    fn netmask_from_ipv4() {
        let cfg = NetworkConfig::ipv4(
            [192, 168, 1, 100],
            [255, 255, 255, 0],
            [192, 168, 1, 1],
        );
        assert_eq!(cfg.netmask(), Some([255u8, 255, 255, 0]));
    }

    #[test]
    fn gateway_from_ipv4() {
        let cfg = NetworkConfig::ipv4(
            [192, 168, 1, 100],
            [255, 255, 255, 0],
            [192, 168, 1, 1],
        );
        assert_eq!(cfg.gateway(), Some([192u8, 168, 1, 1]));
    }

    #[test]
    fn broadcast_calculation() {
        // 192.168.1.100 / 255.255.255.0 → 192.168.1.255
        let cfg = NetworkConfig::ipv4(
            [192, 168, 1, 100],
            [255, 255, 255, 0],
            [192, 168, 1, 1],
        );
        assert_eq!(cfg.broadcast(), Some([192u8, 168, 1, 255]));
    }

    #[test]
    fn netmask_none_for_invalid() {
        assert_eq!(NetworkConfig::invalid().netmask(), None);
    }

    #[test]
    fn default_is_invalid() {
        assert_eq!(NetworkConfig::default(), NetworkConfig::Invalid);
    }

    #[test]
    fn equality() {
        let a = NetworkConfig::ipv4([10, 0, 0, 1], [255, 0, 0, 0], [10, 0, 0, 1]);
        let b = NetworkConfig::ipv4([10, 0, 0, 1], [255, 0, 0, 0], [10, 0, 0, 1]);
        let c = NetworkConfig::ipv4([10, 0, 0, 2], [255, 0, 0, 0], [10, 0, 0, 1]);
        assert_eq!(a, b);
        assert_ne!(a, c);
    }
}
