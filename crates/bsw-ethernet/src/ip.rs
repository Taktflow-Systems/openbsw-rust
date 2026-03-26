//! IP address types — IPv4 and optionally IPv6.

/// IP address family.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddressFamily {
    /// IPv4.
    IPv4,
    /// IPv6 (only available with the `ipv6` feature).
    #[cfg(feature = "ipv6")]
    IPv6,
}

/// IP address — IPv4 or IPv6.
///
/// Bytes are always stored in network byte order (big-endian).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum IpAddress {
    /// IPv4 address stored as 4 bytes in network byte order.
    V4([u8; 4]),
    /// IPv6 address stored as 16 bytes in network byte order.
    #[cfg(feature = "ipv6")]
    V6([u8; 16]),
}

impl IpAddress {
    // ── factory ──────────────────────────────────────────────────────────────

    /// Create from individual octets.
    #[inline]
    pub const fn ipv4(a: u8, b: u8, c: u8, d: u8) -> Self {
        Self::V4([a, b, c, d])
    }

    /// Create from a big-endian `u32` (e.g. `0xC0A80001` = 192.168.0.1).
    #[inline]
    pub const fn ipv4_from_u32(addr: u32) -> Self {
        Self::V4(addr.to_be_bytes())
    }

    /// Create from a 4-byte array in network byte order.
    #[inline]
    pub const fn ipv4_from_bytes(bytes: &[u8; 4]) -> Self {
        Self::V4([bytes[0], bytes[1], bytes[2], bytes[3]])
    }

    /// Create an IPv6 address from a 16-byte array in network byte order.
    #[cfg(feature = "ipv6")]
    #[inline]
    pub const fn ipv6(bytes: [u8; 16]) -> Self {
        Self::V6(bytes)
    }

    /// Create an IPv6 address from four big-endian `u32` words.
    #[cfg(feature = "ipv6")]
    #[inline]
    pub const fn ipv6_from_u32s(a0: u32, a1: u32, a2: u32, a3: u32) -> Self {
        let b0 = a0.to_be_bytes();
        let b1 = a1.to_be_bytes();
        let b2 = a2.to_be_bytes();
        let b3 = a3.to_be_bytes();
        Self::V6([
            b0[0], b0[1], b0[2], b0[3],
            b1[0], b1[1], b1[2], b1[3],
            b2[0], b2[1], b2[2], b2[3],
            b3[0], b3[1], b3[2], b3[3],
        ])
    }

    /// The unspecified address (0.0.0.0 for IPv4).
    #[inline]
    pub const fn unspecified() -> Self {
        Self::V4([0, 0, 0, 0])
    }

    // ── predicates ───────────────────────────────────────────────────────────

    /// Returns `true` if every byte is zero.
    pub fn is_unspecified(&self) -> bool {
        self.as_bytes().iter().all(|&b| b == 0)
    }

    /// Returns `true` if the address is a multicast address.
    ///
    /// IPv4: 224.0.0.0/4 (top nibble == 0xE).
    /// IPv6 (with feature): `ff00::/8`.
    pub const fn is_multicast(&self) -> bool {
        match self {
            Self::V4(b) => (b[0] & 0xF0) == 0xE0,
            #[cfg(feature = "ipv6")]
            Self::V6(b) => b[0] == 0xFF,
        }
    }

    /// Returns `true` if the address is link-local.
    ///
    /// IPv4: 169.254.0.0/16.
    /// IPv6 (with feature): `fe80::/10`.
    pub const fn is_link_local(&self) -> bool {
        match self {
            Self::V4(b) => b[0] == 169 && b[1] == 254,
            #[cfg(feature = "ipv6")]
            Self::V6(b) => b[0] == 0xFE && (b[1] & 0xC0) == 0x80,
        }
    }

    /// Returns `true` if the address is a loopback address.
    ///
    /// IPv4: 127.0.0.0/8 **with last byte non-zero** (i.e. 127.x.x.x where
    /// the last byte ≠ 0; 127.0.0.0 itself is the network address, not a
    /// valid host).
    /// IPv6 (with feature): `::1`.
    pub const fn is_loopback(&self) -> bool {
        match self {
            Self::V4(b) => b[0] == 127 && b[3] != 0,
            #[cfg(feature = "ipv6")]
            Self::V6(b) => {
                b[0] == 0 && b[1] == 0 && b[2] == 0  && b[3] == 0
                    && b[4] == 0 && b[5] == 0 && b[6] == 0 && b[7] == 0
                    && b[8] == 0 && b[9] == 0 && b[10] == 0 && b[11] == 0
                    && b[12] == 0 && b[13] == 0 && b[14] == 0 && b[15] == 1
            }
        }
    }

    /// Returns `true` for IPv4 addresses.
    pub const fn is_ipv4(&self) -> bool {
        matches!(self, Self::V4(_))
    }

    /// Returns `true` for IPv6 addresses.
    pub const fn is_ipv6(&self) -> bool {
        #[cfg(feature = "ipv6")]
        {
            matches!(self, Self::V6(_))
        }
        #[cfg(not(feature = "ipv6"))]
        {
            false
        }
    }

    /// Returns the address family.
    pub const fn family(&self) -> AddressFamily {
        match self {
            Self::V4(_) => AddressFamily::IPv4,
            #[cfg(feature = "ipv6")]
            Self::V6(_) => AddressFamily::IPv6,
        }
    }

    // ── conversions ──────────────────────────────────────────────────────────

    /// For IPv4 addresses, returns the big-endian `u32` representation.
    pub const fn to_ipv4_u32(&self) -> Option<u32> {
        match self {
            Self::V4(b) => Some(u32::from_be_bytes([b[0], b[1], b[2], b[3]])),
            #[cfg(feature = "ipv6")]
            Self::V6(_) => None,
        }
    }

    /// Returns the address as a byte slice (4 bytes for IPv4, 16 for IPv6).
    pub fn as_bytes(&self) -> &[u8] {
        match self {
            Self::V4(b) => b,
            #[cfg(feature = "ipv6")]
            Self::V6(b) => b,
        }
    }
}

impl Default for IpAddress {
    fn default() -> Self {
        Self::unspecified()
    }
}

impl PartialOrd for IpAddress {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for IpAddress {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        match (self, other) {
            (Self::V4(a), Self::V4(b)) => a.cmp(b),
            #[cfg(feature = "ipv6")]
            (Self::V6(a), Self::V6(b)) => a.cmp(b),
            // IPv4 sorts before IPv6
            #[cfg(feature = "ipv6")]
            (Self::V4(_), Self::V6(_)) => core::cmp::Ordering::Less,
            #[cfg(feature = "ipv6")]
            (Self::V6(_), Self::V4(_)) => core::cmp::Ordering::Greater,
        }
    }
}

// ── network utility ───────────────────────────────────────────────────────────

/// Returns `true` if `addr1` and `addr2` share the same `/prefix_len` network.
///
/// Returns `false` if:
/// * the addresses are from different families, or
/// * either address is unspecified.
///
/// Returns `true` if `prefix_len == 0` (don't-care).
pub fn is_network_local(addr1: &IpAddress, addr2: &IpAddress, prefix_len: u8) -> bool {
    if addr1.family() != addr2.family() {
        return false;
    }
    if addr1.is_unspecified() || addr2.is_unspecified() {
        return false;
    }
    if prefix_len == 0 {
        return true;
    }

    let a = addr1.as_bytes();
    let b = addr2.as_bytes();
    let total_bits = prefix_len as usize;
    let full_bytes = total_bits / 8;
    let remainder = total_bits % 8;

    // All full bytes must match.
    if a[..full_bytes] != b[..full_bytes] {
        return false;
    }
    // Check the partial byte (if any).
    if remainder != 0 {
        let mask = 0xFF_u8 << (8 - remainder);
        if (a[full_bytes] & mask) != (b[full_bytes] & mask) {
            return false;
        }
    }
    true
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ipv4_creation() {
        let addr = IpAddress::ipv4(192, 168, 1, 1);
        assert_eq!(addr, IpAddress::V4([192, 168, 1, 1]));
    }

    #[test]
    fn ipv4_from_u32() {
        let addr = IpAddress::ipv4_from_u32(0xC0A8_0001);
        assert_eq!(addr, IpAddress::ipv4(192, 168, 0, 1));
    }

    #[test]
    fn ipv4_from_bytes() {
        let bytes = [10u8, 0, 0, 1];
        let addr = IpAddress::ipv4_from_bytes(&bytes);
        assert_eq!(addr, IpAddress::ipv4(10, 0, 0, 1));
    }

    #[test]
    fn unspecified_is_zero() {
        let addr = IpAddress::unspecified();
        assert_eq!(addr, IpAddress::V4([0, 0, 0, 0]));
    }

    #[test]
    fn is_unspecified() {
        assert!(IpAddress::unspecified().is_unspecified());
        assert!(!IpAddress::ipv4(1, 0, 0, 0).is_unspecified());
    }

    #[test]
    fn is_multicast_ipv4() {
        assert!(IpAddress::ipv4(224, 0, 0, 1).is_multicast());
        assert!(IpAddress::ipv4(239, 255, 255, 255).is_multicast());
        assert!(!IpAddress::ipv4(192, 168, 1, 1).is_multicast());
        assert!(!IpAddress::ipv4(223, 255, 255, 255).is_multicast());
    }

    #[test]
    fn is_link_local_ipv4() {
        assert!(IpAddress::ipv4(169, 254, 1, 1).is_link_local());
        assert!(IpAddress::ipv4(169, 254, 0, 1).is_link_local());
        assert!(!IpAddress::ipv4(10, 0, 0, 1).is_link_local());
        assert!(!IpAddress::ipv4(192, 168, 1, 1).is_link_local());
    }

    #[test]
    fn is_loopback_ipv4() {
        assert!(IpAddress::ipv4(127, 0, 0, 1).is_loopback());
        assert!(IpAddress::ipv4(127, 1, 2, 3).is_loopback());
        // last byte == 0 → not a valid host loopback
        assert!(!IpAddress::ipv4(127, 0, 0, 0).is_loopback());
        assert!(!IpAddress::ipv4(192, 168, 1, 1).is_loopback());
    }

    #[test]
    fn is_ipv4_true() {
        assert!(IpAddress::ipv4(1, 2, 3, 4).is_ipv4());
        assert!(!IpAddress::ipv4(1, 2, 3, 4).is_ipv6());
    }

    #[test]
    fn family_ipv4() {
        assert_eq!(IpAddress::ipv4(1, 2, 3, 4).family(), AddressFamily::IPv4);
    }

    #[test]
    fn to_ipv4_u32() {
        let addr = IpAddress::ipv4(192, 168, 0, 1);
        assert_eq!(addr.to_ipv4_u32(), Some(0xC0A8_0001_u32));
    }

    #[test]
    fn as_bytes_ipv4() {
        let addr = IpAddress::ipv4(10, 20, 30, 40);
        assert_eq!(addr.as_bytes(), &[10u8, 20, 30, 40]);
    }

    #[test]
    fn equality() {
        assert_eq!(IpAddress::ipv4(1, 1, 1, 1), IpAddress::ipv4(1, 1, 1, 1));
        assert_ne!(IpAddress::ipv4(1, 1, 1, 1), IpAddress::ipv4(1, 1, 1, 2));
    }

    #[test]
    fn ordering() {
        assert!(IpAddress::ipv4(1, 0, 0, 0) < IpAddress::ipv4(2, 0, 0, 0));
        assert!(IpAddress::ipv4(192, 168, 1, 1) > IpAddress::ipv4(10, 0, 0, 1));
        assert_eq!(
            IpAddress::ipv4(1, 2, 3, 4).cmp(&IpAddress::ipv4(1, 2, 3, 4)),
            core::cmp::Ordering::Equal
        );
    }

    #[test]
    fn default_is_unspecified() {
        assert_eq!(IpAddress::default(), IpAddress::unspecified());
    }

    #[test]
    fn network_local_same_subnet() {
        let a = IpAddress::ipv4(192, 168, 1, 10);
        let b = IpAddress::ipv4(192, 168, 1, 20);
        assert!(is_network_local(&a, &b, 24));
    }

    #[test]
    fn network_local_different_subnet() {
        let a = IpAddress::ipv4(192, 168, 1, 10);
        let b = IpAddress::ipv4(192, 168, 2, 10);
        assert!(!is_network_local(&a, &b, 24));
    }

    #[test]
    fn network_local_zero_prefix() {
        let a = IpAddress::ipv4(10, 0, 0, 1);
        let b = IpAddress::ipv4(172, 16, 0, 1);
        // prefix_len == 0 → always true (as long as neither is unspecified)
        assert!(is_network_local(&a, &b, 0));
    }

    #[test]
    fn network_local_unspecified_returns_false() {
        let a = IpAddress::unspecified();
        let b = IpAddress::ipv4(192, 168, 1, 1);
        assert!(!is_network_local(&a, &b, 24));
        assert!(!is_network_local(&b, &a, 24));
    }

    // IPv6 tests — only compiled when feature is enabled
    #[cfg(feature = "ipv6")]
    mod ipv6 {
        use super::*;

        #[test]
        fn ipv6_creation() {
            let addr = IpAddress::ipv6([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]);
            assert!(addr.is_ipv6());
        }

        #[test]
        fn is_multicast_ipv6() {
            // ff02::1 (all-nodes multicast)
            let addr = IpAddress::ipv6([0xFF, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]);
            assert!(addr.is_multicast());
            // fe80::1 is NOT multicast
            let ll = IpAddress::ipv6([0xFE, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]);
            assert!(!ll.is_multicast());
        }

        #[test]
        fn is_link_local_ipv6() {
            // fe80::1
            let addr = IpAddress::ipv6([0xFE, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]);
            assert!(addr.is_link_local());
            // 2001::1 is NOT link-local
            let routable = IpAddress::ipv6([0x20, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]);
            assert!(!routable.is_link_local());
        }

        #[test]
        fn is_loopback_ipv6() {
            let loopback = IpAddress::ipv6([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]);
            assert!(loopback.is_loopback());
            let not_loopback = IpAddress::ipv6([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2]);
            assert!(!not_loopback.is_loopback());
        }
    }
}
