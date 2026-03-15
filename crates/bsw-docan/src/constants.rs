//! Protocol constants and enumerations for ISO-TP (ISO 15765-2).

/// ISO-TP frame types encoded in the upper nibble of the PCI byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FrameType {
    /// Single frame — entire message fits in one CAN frame.
    Single = 0,
    /// First frame — first segment of a multi-frame message.
    First = 1,
    /// Consecutive frame — subsequent segment of a multi-frame message.
    Consecutive = 2,
    /// Flow control — receiver feedback to transmitter.
    FlowControl = 3,
}

/// Flow control status byte (lower nibble of FC PCI byte).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FlowStatus {
    /// Receiver ready — continue sending consecutive frames.
    ContinueToSend = 0,
    /// Receiver not yet ready — wait for another FC before sending.
    Wait = 1,
    /// Receiver buffer overflow — abort transmission.
    Overflow = 2,
}

/// Result of a frame codec operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CodecResult {
    /// Operation succeeded.
    Ok,
    /// Frame data is too short or too long for the operation.
    InvalidFrameSize,
    /// Message size field is invalid or unsupported.
    InvalidMessageSize,
    /// Sequence/frame index is out of expected range.
    InvalidFrameIndex,
    /// PCI byte encodes an unknown frame type.
    InvalidFrameType,
}

/// Result returned by a CAN send operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SendResult {
    /// Frame accepted into the TX queue.
    Queued,
    /// Frame accepted but queue is now full.
    QueuedFull,
    /// TX queue is full — frame not accepted.
    Full,
    /// Frame data or parameters are invalid.
    Invalid,
    /// Transmission failed for another reason.
    Failed,
}

/// Protocol-level informational / error messages produced by state machines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProtocolMessage {
    /// No message (normal operation).
    None,
    /// A protocol timer expired.
    Timeout,
    /// Receiver buffer overflow signalled by remote.
    Overflow,
    /// Consecutive frame arrived with unexpected sequence number.
    InvalidSequenceNumber,
    /// Flow control frame is invalid or unexpected.
    InvalidFlowControl,
    /// Too many consecutive WAIT flow controls received.
    WaitCountExceeded,
    /// Buffer allocation failed after all retries.
    AllocationFailed,
    /// CAN send operation failed.
    SendFailed,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_type_repr_values() {
        assert_eq!(FrameType::Single as u8, 0);
        assert_eq!(FrameType::First as u8, 1);
        assert_eq!(FrameType::Consecutive as u8, 2);
        assert_eq!(FrameType::FlowControl as u8, 3);
    }

    #[test]
    fn flow_status_repr_values() {
        assert_eq!(FlowStatus::ContinueToSend as u8, 0);
        assert_eq!(FlowStatus::Wait as u8, 1);
        assert_eq!(FlowStatus::Overflow as u8, 2);
    }

    #[test]
    fn codec_result_equality() {
        assert_eq!(CodecResult::Ok, CodecResult::Ok);
        assert_ne!(CodecResult::Ok, CodecResult::InvalidFrameSize);
        assert_ne!(CodecResult::InvalidMessageSize, CodecResult::InvalidFrameIndex);
    }

    #[test]
    fn send_result_debug_format() {
        let s = format!("{:?}", SendResult::Queued);
        assert_eq!(s, "Queued");
        let s = format!("{:?}", SendResult::Full);
        assert_eq!(s, "Full");
    }

    #[test]
    fn protocol_message_equality() {
        assert_eq!(ProtocolMessage::None, ProtocolMessage::None);
        assert_ne!(ProtocolMessage::Timeout, ProtocolMessage::Overflow);
        assert_ne!(ProtocolMessage::SendFailed, ProtocolMessage::AllocationFailed);
    }
}
