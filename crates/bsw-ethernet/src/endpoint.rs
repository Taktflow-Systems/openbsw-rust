//! IP endpoint — address + port pair.

use crate::ip::IpAddress;

/// IP endpoint combining an [`IpAddress`] and an optional port number.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct IpEndpoint {
    address: IpAddress,
    port: Option<u16>,
}

impl IpEndpoint {
    /// Create a new endpoint with the given address and port.
    #[inline]
    pub const fn new(address: IpAddress, port: u16) -> Self {
        Self {
            address,
            port: Some(port),
        }
    }

    /// Create an unset endpoint (unspecified address, no port).
    #[inline]
    pub const fn unset() -> Self {
        Self {
            address: IpAddress::unspecified(),
            port: None,
        }
    }

    /// Returns a reference to the address.
    #[inline]
    pub const fn address(&self) -> &IpAddress {
        &self.address
    }

    /// Set the address.
    #[inline]
    pub fn set_address(&mut self, addr: IpAddress) {
        self.address = addr;
    }

    /// Returns the port, or `None` if not set.
    #[inline]
    pub const fn port(&self) -> Option<u16> {
        self.port
    }

    /// Set the port.
    #[inline]
    pub fn set_port(&mut self, port: u16) {
        self.port = Some(port);
    }

    /// Returns `true` if a port is present **and** the address is not unspecified.
    #[inline]
    pub fn is_set(&self) -> bool {
        self.port.is_some() && !self.address.is_unspecified()
    }

    /// Reset to the unset state.
    #[inline]
    pub fn clear(&mut self) {
        *self = Self::unset();
    }
}

impl Default for IpEndpoint {
    fn default() -> Self {
        Self::unset()
    }
}

impl PartialOrd for IpEndpoint {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for IpEndpoint {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.address
            .cmp(&other.address)
            .then(self.port.cmp(&other.port))
    }
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ip::IpAddress;

    #[test]
    fn new_endpoint() {
        let ep = IpEndpoint::new(IpAddress::ipv4(192, 168, 1, 1), 8080);
        assert_eq!(*ep.address(), IpAddress::ipv4(192, 168, 1, 1));
        assert_eq!(ep.port(), Some(8080));
    }

    #[test]
    fn unset_is_not_set() {
        let ep = IpEndpoint::unset();
        assert!(!ep.is_set());
        assert_eq!(ep.port(), None);
    }

    #[test]
    fn is_set_with_valid_addr_and_port() {
        let ep = IpEndpoint::new(IpAddress::ipv4(10, 0, 0, 1), 443);
        assert!(ep.is_set());
    }

    #[test]
    fn is_set_with_unspecified_addr() {
        let ep = IpEndpoint::new(IpAddress::unspecified(), 80);
        // address is unspecified → not considered "set"
        assert!(!ep.is_set());
    }

    #[test]
    fn set_address() {
        let mut ep = IpEndpoint::unset();
        ep.set_address(IpAddress::ipv4(1, 2, 3, 4));
        assert_eq!(*ep.address(), IpAddress::ipv4(1, 2, 3, 4));
    }

    #[test]
    fn set_port() {
        let mut ep = IpEndpoint::unset();
        ep.set_port(9000);
        assert_eq!(ep.port(), Some(9000));
    }

    #[test]
    fn clear_resets() {
        let mut ep = IpEndpoint::new(IpAddress::ipv4(192, 168, 0, 1), 53);
        ep.clear();
        assert!(!ep.is_set());
        assert_eq!(*ep.address(), IpAddress::unspecified());
        assert_eq!(ep.port(), None);
    }

    #[test]
    fn equality() {
        let ep1 = IpEndpoint::new(IpAddress::ipv4(1, 2, 3, 4), 80);
        let ep2 = IpEndpoint::new(IpAddress::ipv4(1, 2, 3, 4), 80);
        let ep3 = IpEndpoint::new(IpAddress::ipv4(1, 2, 3, 4), 443);
        assert_eq!(ep1, ep2);
        assert_ne!(ep1, ep3);
    }

    #[test]
    fn default_is_unset() {
        assert_eq!(IpEndpoint::default(), IpEndpoint::unset());
    }

    #[test]
    fn ordering() {
        let ep1 = IpEndpoint::new(IpAddress::ipv4(10, 0, 0, 1), 80);
        let ep2 = IpEndpoint::new(IpAddress::ipv4(10, 0, 0, 1), 443);
        let ep3 = IpEndpoint::new(IpAddress::ipv4(192, 168, 1, 1), 80);
        // same address, lower port < higher port
        assert!(ep1 < ep2);
        // lower address sorts first regardless of port
        assert!(ep1 < ep3);
        assert!(ep2 < ep3);
    }
}
