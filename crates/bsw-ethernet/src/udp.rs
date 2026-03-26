//! UDP socket traits and datagram packet type.

use crate::endpoint::IpEndpoint;
use crate::ip::IpAddress;

/// Error codes for UDP socket operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UdpError {
    /// Operation succeeded.
    Ok,
    /// Generic failure.
    NotOk,
    /// No data listener has been registered.
    NoDataListener,
}

/// A UDP datagram packet — a view into caller-owned data with a source/destination endpoint.
///
/// The lifetime `'a` is tied to the payload slice, enabling zero-copy receive paths.
pub struct DatagramPacket<'a> {
    endpoint: IpEndpoint,
    data: &'a [u8],
}

impl<'a> DatagramPacket<'a> {
    /// Create a packet from raw data plus a target address and port.
    #[inline]
    pub fn new(data: &'a [u8], address: IpAddress, port: u16) -> Self {
        Self {
            endpoint: IpEndpoint::new(address, port),
            data,
        }
    }

    /// Create a packet from raw data and a pre-built endpoint.
    #[inline]
    pub const fn with_endpoint(data: &'a [u8], endpoint: IpEndpoint) -> Self {
        Self { endpoint, data }
    }

    /// Returns the payload bytes.
    #[inline]
    pub fn data(&self) -> &[u8] {
        self.data
    }

    /// Returns the payload length (saturates at `u16::MAX`).
    #[inline]
    pub fn length(&self) -> u16 {
        // SAFETY: min(_, u16::MAX) guarantees the value fits in u16.
        #[allow(clippy::cast_possible_truncation)]
        let len = self.data.len().min(u16::MAX as usize) as u16;
        len
    }

    /// Returns a reference to the endpoint (address + port).
    #[inline]
    pub const fn endpoint(&self) -> &IpEndpoint {
        &self.endpoint
    }

    /// Returns a reference to the address part of the endpoint.
    #[inline]
    pub fn address(&self) -> &IpAddress {
        self.endpoint.address()
    }

    /// Returns the port from the endpoint, or `None` if not set.
    #[inline]
    pub fn port(&self) -> Option<u16> {
        self.endpoint.port()
    }
}

impl PartialEq for DatagramPacket<'_> {
    fn eq(&self, other: &Self) -> bool {
        self.endpoint == other.endpoint && self.data == other.data
    }
}

impl Eq for DatagramPacket<'_> {}

impl core::fmt::Debug for DatagramPacket<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DatagramPacket")
            .field("endpoint", &self.endpoint)
            .field("data_len", &self.data.len())
            .finish()
    }
}

/// UDP socket interface — implemented by the platform.
pub trait UdpSocket {
    /// Bind to a local address and port.
    ///
    /// Pass `None` for the address to bind to all local interfaces.
    fn bind(&mut self, addr: Option<&IpAddress>, port: u16) -> UdpError;
    /// Close the socket.
    fn close(&mut self) -> UdpError;
    /// Returns `true` if the socket is bound to a local port.
    fn is_bound(&self) -> bool;
    /// Returns `true` if the socket is closed.
    fn is_closed(&self) -> bool;
    /// Send data to the connected remote endpoint.
    fn send(&mut self, data: &[u8]) -> UdpError;
    /// Send a datagram to the endpoint specified in `packet`.
    fn send_to(&mut self, packet: &DatagramPacket) -> UdpError;
    /// Read up to `buf.len()` bytes; returns number of bytes read.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, UdpError>;
    /// Connect to a remote endpoint (enables `send`).
    fn connect(&mut self, addr: &IpAddress, port: u16) -> UdpError;
    /// Remove the remote endpoint association.
    fn disconnect(&mut self) -> UdpError;
    /// Returns `true` if a remote endpoint has been set via [`connect`](Self::connect).
    fn is_connected(&self) -> bool;
    /// Local IP address.
    fn local_address(&self) -> IpAddress;
    /// Local port.
    fn local_port(&self) -> u16;
}

/// Callback interface for incoming UDP datagrams.
pub trait UdpDataListener {
    /// Called when a datagram arrives.
    fn on_datagram_received(
        &mut self,
        src_addr: &IpAddress,
        src_port: u16,
        dst_addr: &IpAddress,
        length: u16,
    );
}

/// Callback interface for UDP send completion.
pub trait UdpSendListener {
    /// Called when `data` has been transmitted.
    fn on_datagram_sent(&mut self, data: &[u8]);
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ip::IpAddress;

    fn make_addr() -> IpAddress {
        IpAddress::ipv4(192, 168, 1, 1)
    }

    #[test]
    fn datagram_packet_new() {
        let data = [1u8, 2, 3, 4];
        let pkt = DatagramPacket::new(&data, make_addr(), 5000);
        assert_eq!(pkt.data(), &[1u8, 2, 3, 4]);
        assert_eq!(pkt.port(), Some(5000));
        assert_eq!(*pkt.address(), make_addr());
    }

    #[test]
    fn datagram_packet_with_endpoint() {
        let data = [10u8, 20, 30];
        let ep = IpEndpoint::new(make_addr(), 9999);
        let pkt = DatagramPacket::with_endpoint(&data, ep);
        assert_eq!(*pkt.endpoint(), ep);
        assert_eq!(pkt.data(), &[10u8, 20, 30]);
    }

    #[test]
    fn datagram_data_and_length() {
        let data = [0u8; 10];
        let pkt = DatagramPacket::new(&data, make_addr(), 1234);
        assert_eq!(pkt.length(), 10);
        assert_eq!(pkt.data().len(), 10);
    }

    #[test]
    fn datagram_address_and_port() {
        let addr = IpAddress::ipv4(10, 0, 0, 1);
        let pkt = DatagramPacket::new(&[], addr, 53);
        assert_eq!(*pkt.address(), addr);
        assert_eq!(pkt.port(), Some(53));
    }

    #[test]
    fn datagram_equality_same() {
        let data = [1u8, 2, 3];
        let a = DatagramPacket::new(&data, make_addr(), 80);
        let b = DatagramPacket::new(&data, make_addr(), 80);
        assert_eq!(a, b);
    }

    #[test]
    fn datagram_equality_different_data() {
        let d1 = [1u8, 2, 3];
        let d2 = [1u8, 2, 4];
        let a = DatagramPacket::new(&d1, make_addr(), 80);
        let b = DatagramPacket::new(&d2, make_addr(), 80);
        assert_ne!(a, b);
    }

    #[test]
    fn datagram_equality_different_endpoint() {
        let data = [1u8, 2, 3];
        let a = DatagramPacket::new(&data, IpAddress::ipv4(10, 0, 0, 1), 80);
        let b = DatagramPacket::new(&data, IpAddress::ipv4(10, 0, 0, 2), 80);
        assert_ne!(a, b);
    }

    #[test]
    fn udp_error_variants() {
        let variants = [UdpError::Ok, UdpError::NotOk, UdpError::NoDataListener];
        assert_eq!(variants.len(), 3);
    }

    #[test]
    fn udp_error_debug() {
        assert_eq!(format!("{:?}", UdpError::Ok), "Ok");
        assert_eq!(format!("{:?}", UdpError::NoDataListener), "NoDataListener");
    }

    #[test]
    fn udp_error_equality() {
        assert_eq!(UdpError::Ok, UdpError::Ok);
        assert_ne!(UdpError::Ok, UdpError::NotOk);
    }
}
