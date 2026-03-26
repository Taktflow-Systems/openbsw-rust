//! ISO-TP protocol parameters and separation time encoding/decoding.
//!
//! Separation time (`STmin`) is encoded as a single byte per ISO 15765-2 §9.6.5.4:
//! - `0x00`–`0x7F` → 0–127 ms (1 ms resolution)
//! - `0xF1`–`0xF9` → 100–900 µs (100 µs resolution)
//! - `0x80`–`0xF0`, `0xFA`–`0xFF` → reserved, treat as 127 ms (maximum)

/// Decode a separation time byte into microseconds.
///
/// # Examples
/// ```
/// # use bsw_docan::parameters::decode_separation_time;
/// assert_eq!(decode_separation_time(0x00), 0);
/// assert_eq!(decode_separation_time(0x01), 1_000);
/// assert_eq!(decode_separation_time(0xF1), 100);
/// assert_eq!(decode_separation_time(0x80), 127_000); // reserved
/// ```
pub const fn decode_separation_time(encoded: u8) -> u32 {
    match encoded {
        0x00..=0x7F => (encoded as u32) * 1_000,
        0xF1..=0xF9 => (encoded as u32 - 0xF0) * 100,
        // 0x80..=0xF0, 0xFA..=0xFF are reserved → use maximum (127 ms)
        _ => 127_000,
    }
}

/// Encode a microsecond value into the closest valid separation time byte.
///
/// Encoding priority (ISO 15765-2):
/// 1. 0 µs → `0x00`
/// 2. Multiples of 100 µs in range 100–900 → `0xF1`–`0xF9`
/// 3. Otherwise round up to the next millisecond, clamp to 127 ms → `0x00`–`0x7F`
///
/// # Examples
/// ```
/// # use bsw_docan::parameters::encode_separation_time;
/// assert_eq!(encode_separation_time(0), 0x00);
/// assert_eq!(encode_separation_time(100), 0xF1);
/// assert_eq!(encode_separation_time(1_000), 0x01);
/// assert_eq!(encode_separation_time(127_000), 0x7F);
/// assert_eq!(encode_separation_time(200_000), 0x7F); // above max → clamp
/// ```
#[allow(clippy::cast_possible_truncation)] // us/100 is ≤ 9 here, safe to u8
pub const fn encode_separation_time(us: u32) -> u8 {
    if us == 0 {
        0x00
    } else if us <= 900 && us.is_multiple_of(100) {
        // 100–900 µs range in steps of 100 µs; result fits in u8 (1..=9 + 0xF0)
        (us / 100) as u8 + 0xF0
    } else {
        // Round up to next full millisecond, then clamp to 127.
        // Guard against overflow: if us > u32::MAX - 999, it's already >> 127 ms.
        #[allow(clippy::manual_div_ceil)] // div_ceil is not const-stable yet
        let ms = if us > u32::MAX - 999 {
            u32::MAX / 1_000 // guaranteed >> 127, will clamp below
        } else {
            (us + 999) / 1_000
        };
        if ms > 127 {
            0x7F
        } else {
            // ms is ≤ 127, safe to cast
            #[allow(clippy::cast_possible_truncation)]
            { ms as u8 }
        }
    }
}

/// ISO-TP protocol timing and sizing parameters.
///
/// All timeout values are in microseconds. The state machines consume these
/// values directly; no OS timer types are referenced here.
#[derive(Debug, Clone, Copy)]
pub struct Parameters {
    /// How long to wait before retrying a buffer allocation attempt (µs).
    pub wait_allocate_timeout_us: u32,
    /// Maximum wait between consecutive frames before the RX session aborts (µs).
    pub wait_rx_timeout_us: u32,
    /// Maximum wait for the CAN TX-done callback (µs).
    pub wait_tx_callback_timeout_us: u32,
    /// Maximum wait for a flow control frame from the receiver (µs).
    pub wait_flow_control_timeout_us: u32,
    /// Maximum number of buffer allocation retries before sending WAIT FC.
    pub max_allocate_retry_count: u8,
    /// Maximum number of consecutive WAIT flow controls before aborting TX (ISO §6.7.7).
    pub max_flow_control_wait_count: u8,
    /// Minimum separation time between consecutive TX frames (µs).
    pub min_separation_time_us: u32,
    /// Maximum frames per block (block size in FC). 0 = unlimited.
    pub max_block_size: u8,
}

impl Default for Parameters {
    fn default() -> Self {
        Self {
            wait_allocate_timeout_us: 10_000,        // 10 ms
            wait_rx_timeout_us: 1_000_000,           // 1 s
            wait_tx_callback_timeout_us: 100_000,    // 100 ms
            wait_flow_control_timeout_us: 1_000_000, // 1 s
            max_allocate_retry_count: 3,
            max_flow_control_wait_count: 10,
            min_separation_time_us: 0,
            max_block_size: 0, // unlimited
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- decode_separation_time ---

    #[test]
    fn decode_0ms() {
        assert_eq!(decode_separation_time(0x00), 0);
    }

    #[test]
    fn decode_50ms() {
        assert_eq!(decode_separation_time(50), 50_000);
    }

    #[test]
    fn decode_127ms() {
        assert_eq!(decode_separation_time(0x7F), 127_000);
    }

    #[test]
    fn decode_100us() {
        assert_eq!(decode_separation_time(0xF1), 100);
    }

    #[test]
    fn decode_900us() {
        assert_eq!(decode_separation_time(0xF9), 900);
    }

    #[test]
    fn decode_reserved_0x80() {
        // Reserved range → max (127 ms)
        assert_eq!(decode_separation_time(0x80), 127_000);
        assert_eq!(decode_separation_time(0xF0), 127_000);
    }

    #[test]
    fn decode_reserved_0xfa() {
        assert_eq!(decode_separation_time(0xFA), 127_000);
        assert_eq!(decode_separation_time(0xFF), 127_000);
    }

    // --- encode_separation_time ---

    #[test]
    fn encode_0us() {
        assert_eq!(encode_separation_time(0), 0x00);
    }

    #[test]
    fn encode_1ms() {
        assert_eq!(encode_separation_time(1_000), 0x01);
    }

    #[test]
    fn encode_127ms() {
        assert_eq!(encode_separation_time(127_000), 0x7F);
    }

    #[test]
    fn encode_500us() {
        assert_eq!(encode_separation_time(500), 0xF5);
    }

    #[test]
    fn encode_above_127ms() {
        // Any value above 127 ms clamps to 0x7F
        assert_eq!(encode_separation_time(200_000), 0x7F);
        assert_eq!(encode_separation_time(u32::MAX), 0x7F);
    }

    // --- roundtrip ---

    #[test]
    fn roundtrip_separation_time() {
        // All valid ms values (0–127 ms)
        for ms in 0u32..=127 {
            let encoded = encode_separation_time(ms * 1_000);
            let decoded = decode_separation_time(encoded);
            assert_eq!(decoded, ms * 1_000, "roundtrip failed for {ms} ms");
        }
        // All valid sub-ms values (100–900 µs in steps of 100)
        for step in 1u32..=9 {
            let us = step * 100;
            let encoded = encode_separation_time(us);
            let decoded = decode_separation_time(encoded);
            assert_eq!(decoded, us, "roundtrip failed for {us} µs");
        }
    }

    // --- Parameters defaults ---

    #[test]
    fn parameters_default_values() {
        let p = Parameters::default();
        assert_eq!(p.wait_allocate_timeout_us, 10_000);
        assert_eq!(p.wait_rx_timeout_us, 1_000_000);
        assert_eq!(p.wait_tx_callback_timeout_us, 100_000);
        assert_eq!(p.wait_flow_control_timeout_us, 1_000_000);
        assert_eq!(p.max_allocate_retry_count, 3);
        assert_eq!(p.max_flow_control_wait_count, 10);
        assert_eq!(p.min_separation_time_us, 0);
        assert_eq!(p.max_block_size, 0);
    }
}
