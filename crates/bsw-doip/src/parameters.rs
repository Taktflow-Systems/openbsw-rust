//! `DoIP` transport layer configuration parameters — timeouts and buffer sizes.

/// `DoIP` transport layer timeout and buffer configuration.
///
/// All timeout values are in milliseconds.  Defaults follow the ISO 13400-2
/// recommended values where applicable.
#[derive(Debug, Clone, Copy)]
pub struct TransportParameters {
    /// Timeout for routing activation after a new TCP connection (ms).
    ///
    /// If no routing activation request is received within this window the
    /// connection is closed.  ISO 13400-2 calls this `T_TCP_Initial_Inactivity`.
    pub inactivity_timeout_ms: u32,

    /// General inactivity timeout before closing an idle connection (ms).
    ///
    /// ISO 13400-2 calls this `T_TCP_General_Inactivity`.
    pub general_inactivity_timeout_ms: u32,

    /// Timeout waiting for an alive-check response (ms).
    ///
    /// ISO 13400-2 calls this `T_TCP_Alive_Check`.
    pub alive_check_timeout_ms: u32,

    /// Maximum diagnostic message payload size accepted (bytes).
    pub max_payload_size: u32,

    /// TCP receive buffer size (bytes).
    ///
    /// Should be at least `HEADER_SIZE + max_payload_size`.
    pub tcp_rx_buffer_size: u32,
}

impl Default for TransportParameters {
    fn default() -> Self {
        Self {
            inactivity_timeout_ms: 2_000,
            general_inactivity_timeout_ms: 300_000,
            alive_check_timeout_ms: 500,
            max_payload_size: 4_096,
            tcp_rx_buffer_size: 4_104, // 8-byte header + 4096 payload
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_values() {
        let p = TransportParameters::default();
        assert_eq!(p.inactivity_timeout_ms, 2_000);
        assert_eq!(p.general_inactivity_timeout_ms, 300_000);
        assert_eq!(p.alive_check_timeout_ms, 500);
        assert_eq!(p.max_payload_size, 4_096);
        assert_eq!(p.tcp_rx_buffer_size, 4_104);
    }

    #[test]
    fn custom_values() {
        let p = TransportParameters {
            inactivity_timeout_ms: 1_000,
            general_inactivity_timeout_ms: 60_000,
            alive_check_timeout_ms: 250,
            max_payload_size: 8_192,
            tcp_rx_buffer_size: 8_200,
        };
        assert_eq!(p.inactivity_timeout_ms, 1_000);
        assert_eq!(p.tcp_rx_buffer_size, 8_200);
    }

    #[test]
    fn debug_format() {
        let p = TransportParameters::default();
        let s = format!("{p:?}");
        assert!(s.contains("TransportParameters"));
        assert!(s.contains("2000"));
    }

    #[test]
    fn clone() {
        let p = TransportParameters::default();
        let q = p;
        assert_eq!(p.inactivity_timeout_ms, q.inactivity_timeout_ms);
    }

    #[test]
    fn copy() {
        fn takes_copy(p: TransportParameters) -> u32 {
            p.max_payload_size
        }
        let p = TransportParameters::default();
        // If TransportParameters is Copy, we can pass it without moving.
        assert_eq!(takes_copy(p), 4_096);
        // p is still usable here because it was copied
        assert_eq!(p.alive_check_timeout_ms, 500);
    }
}
