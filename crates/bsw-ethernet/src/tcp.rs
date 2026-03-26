//! TCP socket traits — platform implementations provide the concrete types.

use crate::ip::IpAddress;

/// Error codes for TCP socket operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TcpError {
    /// Operation succeeded.
    Ok,
    /// Generic failure.
    NotOk,
    /// Socket is not open.
    NotOpen,
    /// Transmit buffer is full.
    NoMoreBuffer,
    /// Data flush is required before more data can be sent.
    Flush,
}

/// Reason a TCP connection was closed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionCloseReason {
    /// The remote peer closed the connection gracefully.
    ClosedByPeer,
    /// The connection was reset.
    Reset,
    /// The connection timed out.
    Timeout,
    /// Reason is not known.
    Unknown,
}

/// Result of a TCP send operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SendResult {
    /// Data was transmitted immediately.
    DataSent,
    /// Data was accepted into the send queue but not yet transmitted.
    DataQueued,
}

/// TCP socket interface — implemented by the platform.
pub trait TcpSocket {
    /// Bind the socket to a local address and port.
    fn bind(&mut self, addr: &IpAddress, port: u16) -> TcpError;
    /// Initiate a connection to a remote endpoint.
    fn connect(&mut self, addr: &IpAddress, port: u16) -> TcpError;
    /// Close the connection gracefully.
    fn close(&mut self) -> TcpError;
    /// Abort the connection immediately.
    fn abort(&mut self);
    /// Flush any buffered outgoing data.
    fn flush(&mut self);
    /// Send data over the connection.
    fn send(&mut self, data: &[u8]) -> TcpError;
    /// Read up to `buf.len()` bytes; returns number of bytes read.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, TcpError>;
    /// Number of bytes available for reading.
    fn available(&self) -> usize;
    /// Returns `true` if the socket is in the closed state.
    fn is_closed(&self) -> bool;
    /// Returns `true` if the connection is fully established.
    fn is_established(&self) -> bool;
    /// Remote IP address.
    fn remote_address(&self) -> IpAddress;
    /// Local IP address.
    fn local_address(&self) -> IpAddress;
    /// Remote port number.
    fn remote_port(&self) -> u16;
    /// Local port number.
    fn local_port(&self) -> u16;
}

/// TCP server (listening) socket interface — implemented by the platform.
pub trait TcpServerSocket {
    /// Bind the server socket to a local address and port.
    fn bind(&mut self, addr: &IpAddress, port: u16) -> TcpError;
    /// Stop listening and close the server socket.
    fn close(&mut self) -> TcpError;
    /// Returns `true` if the server socket is closed.
    fn is_closed(&self) -> bool;
    /// Local port this server is listening on.
    fn local_port(&self) -> u16;
}

/// Callback interface for incoming TCP data.
pub trait TcpDataListener {
    /// Called when `length` bytes have been received.
    fn on_data_received(&mut self, length: u16);
    /// Called when the connection is closed for the given reason.
    fn on_connection_closed(&mut self, reason: ConnectionCloseReason);
}

/// Callback interface for TCP send completion.
pub trait TcpSendListener {
    /// Called when `length` bytes have been sent with the given result.
    fn on_data_sent(&mut self, length: u16, result: SendResult);
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tcp_error_variants() {
        let variants = [
            TcpError::Ok,
            TcpError::NotOk,
            TcpError::NotOpen,
            TcpError::NoMoreBuffer,
            TcpError::Flush,
        ];
        assert_eq!(variants.len(), 5);
    }

    #[test]
    fn connection_close_reason_variants() {
        let variants = [
            ConnectionCloseReason::ClosedByPeer,
            ConnectionCloseReason::Reset,
            ConnectionCloseReason::Timeout,
            ConnectionCloseReason::Unknown,
        ];
        assert_eq!(variants.len(), 4);
    }

    #[test]
    fn send_result_variants() {
        let variants = [SendResult::DataSent, SendResult::DataQueued];
        assert_eq!(variants.len(), 2);
    }

    #[test]
    fn tcp_error_debug_format() {
        assert_eq!(format!("{:?}", TcpError::Ok), "Ok");
        assert_eq!(format!("{:?}", TcpError::NoMoreBuffer), "NoMoreBuffer");
    }

    #[test]
    fn tcp_error_equality() {
        assert_eq!(TcpError::Ok, TcpError::Ok);
        assert_ne!(TcpError::Ok, TcpError::NotOk);
        assert_ne!(TcpError::NotOpen, TcpError::Flush);
    }
}
