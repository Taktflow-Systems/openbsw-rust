//! Timed, platform-independent ISO-TP sessions (packages E10-E12).
//!
//! All deadlines use injected [`Instant`] values. The sessions own fixed
//! buffers and expose frame actions; platform adapters decide how CAN frames
//! are transmitted and when confirmations arrive.

use bsw_time::{Duration, Instant};
use bsw_transport::TransportMessage;

use crate::addressing::{Addressing, ConnectionInfo};
use crate::codec::{
    decode_frame, encode_consecutive_frame, encode_first_frame, encode_flow_control,
    encode_single_frame, DecodedFrame,
};
use crate::constants::FlowStatus;
use crate::parameters::{decode_separation_time, Parameters};

/// Maximum CAN/CAN-FD payload represented by the platform-independent core.
pub const MAX_CAN_PAYLOAD: usize = 64;

/// ISO-TP timer whose expiry aborted a session.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimerKind {
    /// Receiver buffer-allocation response timer.
    NBr,
    /// Receiver consecutive-frame timer.
    NCr,
    /// Sender CAN-confirmation timer.
    NAs,
    /// Sender flow-control timer.
    NBs,
    /// Sender consecutive-frame submission timer.
    NCs,
}

/// Session failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionError {
    /// Another message is active.
    Busy,
    /// Message cannot fit configured fixed storage.
    MessageTooLong,
    /// Frame was malformed or used an unexpected type/state.
    MalformedFrame,
    /// Consecutive-frame sequence number was not the expected modulo-16 value.
    WrongSequence,
    /// Receiver has insufficient capacity or peer sent overflow.
    Overflow,
    /// A named protocol timer expired.
    Timeout(TimerKind),
    /// Peer exceeded the configured flow-control WAIT limit.
    WaitCountExceeded,
    /// Application cancelled the transfer.
    Cancelled,
    /// CAN submission/confirmation failed.
    LinkFailure,
    /// Address byte did not match this connection.
    AddressMismatch,
}

/// One encoded CAN payload action.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameAction {
    bytes: [u8; MAX_CAN_PAYLOAD],
    len: usize,
}

impl FrameAction {
    /// Encoded CAN data bytes.
    pub fn bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

/// Connection-specific transport settings.
#[derive(Debug, Clone, Copy)]
pub struct ConnectionConfig {
    /// Logical/CAN address mapping.
    pub connection: ConnectionInfo,
    /// ISO-TP address-byte format.
    pub addressing: Addressing,
    /// CAN payload size (8 for classic, one of the supported FD lengths up to 64).
    pub frame_size: u8,
    /// Padding byte.
    pub filler_byte: u8,
    /// Receiver block size; zero is unlimited.
    pub block_size: u8,
    /// Protocol timers and limits.
    pub parameters: Parameters,
}

impl ConnectionConfig {
    fn valid_frame_size(self) -> bool {
        matches!(self.frame_size, 8 | 12 | 16 | 20 | 24 | 32 | 48 | 64)
            && usize::from(self.frame_size) > self.addressing.pci_offset() + 2
    }
}

/// Transmit session state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxSessionState {
    /// No active transfer.
    Idle,
    /// A frame can be emitted by [`TxSession::poll`].
    Ready,
    /// Awaiting CAN completion (N_As).
    WaitConfirmation,
    /// Awaiting receiver flow control (N_Bs).
    WaitFlowControl,
    /// Respecting STmin before the next CF, bounded by N_Cs.
    WaitSeparation,
    /// All payload bytes were confirmed.
    Complete,
    /// Transfer aborted.
    Failed(SessionError),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum SentFrame {
    Single,
    First,
    Consecutive,
}

/// Fixed-buffer ISO-TP transmitter.
pub struct TxSession<const PAYLOAD: usize> {
    config: ConnectionConfig,
    payload: [u8; PAYLOAD],
    len: usize,
    offset: usize,
    sequence: u8,
    block_remaining: u8,
    wait_count: u8,
    separation: Duration,
    earliest: Instant,
    deadline: Instant,
    sent: Option<SentFrame>,
    state: TxSessionState,
}

impl<const PAYLOAD: usize> TxSession<PAYLOAD> {
    /// Create an idle transmitter.
    pub const fn new(config: ConnectionConfig) -> Self {
        Self {
            config,
            payload: [0; PAYLOAD],
            len: 0,
            offset: 0,
            sequence: 1,
            block_remaining: 0,
            wait_count: 0,
            separation: Duration::ZERO,
            earliest: Instant::from_nanos(0),
            deadline: Instant::from_nanos(0),
            sent: None,
            state: TxSessionState::Idle,
        }
    }

    /// Current transmit state.
    pub const fn state(&self) -> TxSessionState {
        self.state
    }

    /// Start a new message, replacing a terminal/idle session.
    pub fn start(&mut self, payload: &[u8]) -> Result<(), SessionError> {
        if matches!(
            self.state,
            TxSessionState::Ready
                | TxSessionState::WaitConfirmation
                | TxSessionState::WaitFlowControl
                | TxSessionState::WaitSeparation
        ) {
            return Err(SessionError::Busy);
        }
        if payload.is_empty() || payload.len() > PAYLOAD || !self.config.valid_frame_size() {
            return Err(SessionError::MessageTooLong);
        }
        self.payload[..payload.len()].copy_from_slice(payload);
        self.len = payload.len();
        self.offset = 0;
        self.sequence = 1;
        self.block_remaining = 0;
        self.wait_count = 0;
        self.sent = None;
        self.state = TxSessionState::Ready;
        Ok(())
    }

    /// Emit the next due frame or apply a protocol timeout.
    pub fn poll(&mut self, now: Instant) -> Result<Option<FrameAction>, SessionError> {
        match self.state {
            TxSessionState::WaitConfirmation | TxSessionState::WaitFlowControl
                if now.is_at_or_after(self.deadline) =>
            {
                let timer = if self.state == TxSessionState::WaitConfirmation {
                    TimerKind::NAs
                } else {
                    TimerKind::NBs
                };
                return self.fail(SessionError::Timeout(timer));
            }
            TxSessionState::WaitSeparation if now.is_at_or_after(self.deadline) => {
                return self.fail(SessionError::Timeout(TimerKind::NCs));
            }
            TxSessionState::WaitSeparation if now.is_at_or_after(self.earliest) => {
                self.state = TxSessionState::Ready;
            }
            _ => {}
        }
        if self.state != TxSessionState::Ready {
            return Ok(None);
        }
        self.encode_next(now).map(Some)
    }

    /// Report completion of the most recently emitted CAN frame.
    pub fn confirm(&mut self, success: bool, now: Instant) -> Result<(), SessionError> {
        if self.state != TxSessionState::WaitConfirmation {
            return self.fail(SessionError::MalformedFrame);
        }
        if !success {
            return self.fail(SessionError::LinkFailure);
        }
        let Some(sent) = self.sent.take() else {
            return self.fail(SessionError::MalformedFrame);
        };
        match sent {
            SentFrame::Single => self.state = TxSessionState::Complete,
            SentFrame::First => {
                self.state = TxSessionState::WaitFlowControl;
                self.deadline = add_us(now, self.config.parameters.wait_flow_control_timeout_us);
            }
            SentFrame::Consecutive if self.offset == self.len => {
                self.state = TxSessionState::Complete;
            }
            SentFrame::Consecutive if self.block_remaining == 1 => {
                self.block_remaining = 0;
                self.state = TxSessionState::WaitFlowControl;
                self.deadline = add_us(now, self.config.parameters.wait_flow_control_timeout_us);
            }
            SentFrame::Consecutive => {
                if self.block_remaining > 1 {
                    self.block_remaining -= 1;
                }
                self.wait_for_separation(now);
            }
        }
        Ok(())
    }

    /// Apply an incoming flow-control frame.
    pub fn flow_control(
        &mut self,
        status: FlowStatus,
        block_size: u8,
        st_min: u8,
        now: Instant,
    ) -> Result<(), SessionError> {
        if self.state != TxSessionState::WaitFlowControl {
            return self.fail(SessionError::MalformedFrame);
        }
        match status {
            FlowStatus::ContinueToSend => {
                self.block_remaining = block_size;
                self.wait_count = 0;
                let peer_us = decode_separation_time(st_min);
                let selected = peer_us.max(self.config.parameters.min_separation_time_us);
                self.separation = duration_us(selected);
                self.wait_for_separation(now);
                Ok(())
            }
            FlowStatus::Wait => {
                self.wait_count = self.wait_count.saturating_add(1);
                if self.wait_count >= self.config.parameters.max_flow_control_wait_count {
                    self.fail(SessionError::WaitCountExceeded)
                } else {
                    self.deadline =
                        add_us(now, self.config.parameters.wait_flow_control_timeout_us);
                    Ok(())
                }
            }
            FlowStatus::Overflow => self.fail(SessionError::Overflow),
        }
    }

    /// Abort the transfer; the next `start` recovers the session.
    pub fn cancel(&mut self) {
        self.state = TxSessionState::Failed(SessionError::Cancelled);
    }

    fn encode_next(&mut self, now: Instant) -> Result<FrameAction, SessionError> {
        let frame_size = usize::from(self.config.frame_size);
        let codec = self.config.addressing.codec(self.config.filler_byte);
        let mut bytes = [self.config.filler_byte; MAX_CAN_PAYLOAD];
        if !self
            .config
            .addressing
            .write_prefix(&mut bytes[..frame_size])
        {
            return self.fail(SessionError::MessageTooLong);
        }
        let (len, sent) = if self.offset == 0 {
            if let Ok(len) =
                encode_single_frame(&mut bytes[..frame_size], &self.payload[..self.len], &codec)
            {
                self.offset = self.len;
                (len, SentFrame::Single)
            } else {
                let header = if self.len <= 4095 { 2 } else { 6 };
                let capacity = frame_size.saturating_sub(codec.pci_offset + header);
                if capacity == 0 {
                    return self.fail(SessionError::MessageTooLong);
                }
                let count = capacity.min(self.len);
                let len = encode_first_frame(
                    &mut bytes[..frame_size],
                    self.len as u32,
                    &self.payload[..count],
                    &codec,
                )
                .map_err(|_| SessionError::MessageTooLong)?;
                self.offset = count;
                (len, SentFrame::First)
            }
        } else {
            let capacity = frame_size.saturating_sub(codec.pci_offset + 1);
            let count = capacity.min(self.len - self.offset);
            let len = encode_consecutive_frame(
                &mut bytes[..frame_size],
                self.sequence,
                &self.payload[self.offset..self.offset + count],
                &codec,
            )
            .map_err(|_| SessionError::MessageTooLong)?;
            self.sequence = self.sequence.wrapping_add(1) & 0x0f;
            self.offset += count;
            (len, SentFrame::Consecutive)
        };
        self.sent = Some(sent);
        self.state = TxSessionState::WaitConfirmation;
        self.deadline = add_us(now, self.config.parameters.wait_tx_callback_timeout_us);
        Ok(FrameAction { bytes, len })
    }

    fn wait_for_separation(&mut self, now: Instant) {
        self.earliest = now.wrapping_add(self.separation);
        self.deadline = add_us(now, self.config.parameters.wait_consecutive_send_timeout_us);
        self.state = TxSessionState::WaitSeparation;
    }

    fn fail<T>(&mut self, error: SessionError) -> Result<T, SessionError> {
        self.state = TxSessionState::Failed(error);
        Err(error)
    }
}

/// Receive session state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxSessionState {
    /// Waiting for a first/single frame.
    Idle,
    /// Waiting to retry upper-layer allocation (N_Br).
    WaitAllocation,
    /// Waiting for the next consecutive frame (N_Cr).
    WaitConsecutive,
    /// Complete message is available.
    Complete,
    /// Transfer aborted.
    Failed(SessionError),
}

/// Action returned by the receiver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReceiveAction {
    /// No frame needs sending.
    None,
    /// Send this flow-control payload.
    FlowControl(FrameAction),
    /// A message became available through [`RxSession::take_message`].
    Complete,
}

/// Fixed-buffer ISO-TP receiver with allocation retry modeling.
pub struct RxSession<const PAYLOAD: usize> {
    config: ConnectionConfig,
    message: TransportMessage<PAYLOAD>,
    expected: usize,
    offset: usize,
    next_sequence: u8,
    block_count: u8,
    allocation_retries: u8,
    deadline: Instant,
    state: RxSessionState,
}

impl<const PAYLOAD: usize> RxSession<PAYLOAD> {
    /// Create an idle receiver.
    pub const fn new(config: ConnectionConfig) -> Self {
        Self {
            config,
            message: TransportMessage::new(),
            expected: 0,
            offset: 0,
            next_sequence: 1,
            block_count: 0,
            allocation_retries: 0,
            deadline: Instant::from_nanos(0),
            state: RxSessionState::Idle,
        }
    }

    /// Current receive state.
    pub const fn state(&self) -> RxSessionState {
        self.state
    }

    /// Process an incoming non-flow-control frame.
    pub fn receive(
        &mut self,
        frame: &[u8],
        now: Instant,
        allocation_available: bool,
    ) -> Result<ReceiveAction, SessionError> {
        if !self.config.addressing.accepts(frame) {
            return Err(SessionError::AddressMismatch);
        }
        let codec = self.config.addressing.codec(self.config.filler_byte);
        match decode_frame(frame, &codec).map_err(|_| SessionError::MalformedFrame)? {
            DecodedFrame::Single(single) => {
                if !allocation_available || usize::from(single.data_length) > PAYLOAD {
                    return self.fail(SessionError::Overflow);
                }
                self.begin_message();
                self.message
                    .append(single.data)
                    .map_err(|_| SessionError::Overflow)?;
                self.expected = usize::from(single.data_length);
                self.offset = self.expected;
                self.state = RxSessionState::Complete;
                Ok(ReceiveAction::Complete)
            }
            DecodedFrame::First(first) => {
                self.receive_first(first.message_length, first.data, now, allocation_available)
            }
            DecodedFrame::Consecutive(frame) => {
                self.receive_consecutive(frame.sequence_number, frame.data, now)
            }
            DecodedFrame::FlowControl(_) => Err(SessionError::MalformedFrame),
        }
    }

    /// Drive N_Br/N_Cr timers and allocation retries.
    pub fn poll(
        &mut self,
        now: Instant,
        allocation_available: bool,
    ) -> Result<ReceiveAction, SessionError> {
        if !now.is_at_or_after(self.deadline) {
            return Ok(ReceiveAction::None);
        }
        match self.state {
            RxSessionState::WaitAllocation => {
                if allocation_available {
                    self.state = RxSessionState::WaitConsecutive;
                    self.deadline = add_us(now, self.config.parameters.wait_rx_timeout_us);
                    self.flow_control(FlowStatus::ContinueToSend)
                } else {
                    self.allocation_retries = self.allocation_retries.saturating_add(1);
                    if self.allocation_retries >= self.config.parameters.max_allocate_retry_count {
                        self.state = RxSessionState::Failed(SessionError::Overflow);
                        self.flow_control(FlowStatus::Overflow)
                    } else {
                        self.deadline =
                            add_us(now, self.config.parameters.wait_allocate_timeout_us);
                        self.flow_control(FlowStatus::Wait)
                    }
                }
            }
            RxSessionState::WaitConsecutive => self.fail(SessionError::Timeout(TimerKind::NCr)),
            _ => Ok(ReceiveAction::None),
        }
    }

    /// Move a completed transport message out and recover to idle.
    pub fn take_message(&mut self) -> Option<TransportMessage<PAYLOAD>> {
        if self.state != RxSessionState::Complete {
            return None;
        }
        self.state = RxSessionState::Idle;
        Some(core::mem::take(&mut self.message))
    }

    /// Abort and return to a terminal state; the next first frame recovers.
    pub fn abort(&mut self) {
        self.state = RxSessionState::Failed(SessionError::Cancelled);
    }

    fn receive_first(
        &mut self,
        message_length: u32,
        data: &[u8],
        now: Instant,
        allocation_available: bool,
    ) -> Result<ReceiveAction, SessionError> {
        let expected = usize::try_from(message_length).map_err(|_| SessionError::Overflow)?;
        if expected > PAYLOAD || data.len() > expected {
            self.state = RxSessionState::Failed(SessionError::Overflow);
            return self.flow_control(FlowStatus::Overflow);
        }
        self.begin_message();
        self.message
            .append(data)
            .map_err(|_| SessionError::Overflow)?;
        self.expected = expected;
        self.offset = data.len();
        if allocation_available {
            self.state = RxSessionState::WaitConsecutive;
            self.deadline = add_us(now, self.config.parameters.wait_rx_timeout_us);
            self.flow_control(FlowStatus::ContinueToSend)
        } else {
            self.state = RxSessionState::WaitAllocation;
            self.deadline = add_us(now, self.config.parameters.wait_allocate_timeout_us);
            self.flow_control(FlowStatus::Wait)
        }
    }

    fn receive_consecutive(
        &mut self,
        sequence: u8,
        data: &[u8],
        now: Instant,
    ) -> Result<ReceiveAction, SessionError> {
        if self.state != RxSessionState::WaitConsecutive {
            return self.fail(SessionError::MalformedFrame);
        }
        if sequence != self.next_sequence {
            return self.fail(SessionError::WrongSequence);
        }
        self.next_sequence = self.next_sequence.wrapping_add(1) & 0x0f;
        let remaining = self.expected - self.offset;
        let count = remaining.min(data.len());
        self.message
            .append(&data[..count])
            .map_err(|_| SessionError::Overflow)?;
        self.offset += count;
        if self.offset == self.expected {
            self.state = RxSessionState::Complete;
            return Ok(ReceiveAction::Complete);
        }
        self.block_count = self.block_count.saturating_add(1);
        self.deadline = add_us(now, self.config.parameters.wait_rx_timeout_us);
        if self.config.block_size != 0 && self.block_count == self.config.block_size {
            self.block_count = 0;
            self.flow_control(FlowStatus::ContinueToSend)
        } else {
            Ok(ReceiveAction::None)
        }
    }

    fn begin_message(&mut self) {
        self.message.clear();
        self.message
            .set_source_address(self.config.connection.transport.source);
        self.message
            .set_target_address(self.config.connection.transport.target);
        self.expected = 0;
        self.offset = 0;
        self.next_sequence = 1;
        self.block_count = 0;
        self.allocation_retries = 0;
    }

    fn flow_control(&self, status: FlowStatus) -> Result<ReceiveAction, SessionError> {
        let size = usize::from(self.config.frame_size);
        let codec = self.config.addressing.codec(self.config.filler_byte);
        let mut bytes = [self.config.filler_byte; MAX_CAN_PAYLOAD];
        self.config.addressing.write_prefix(&mut bytes[..size]);
        let len = encode_flow_control(
            &mut bytes[..size],
            status,
            self.config.block_size,
            crate::parameters::encode_separation_time(
                self.config.parameters.min_separation_time_us,
            ),
            &codec,
        )
        .map_err(|_| SessionError::MalformedFrame)?;
        Ok(ReceiveAction::FlowControl(FrameAction { bytes, len }))
    }

    fn fail<T>(&mut self, error: SessionError) -> Result<T, SessionError> {
        self.state = RxSessionState::Failed(error);
        Err(error)
    }
}

fn duration_us(value: u32) -> Duration {
    Duration::from_micros(u64::from(value)).unwrap_or(Duration::from_nanos(u64::MAX))
}

fn add_us(now: Instant, value: u32) -> Instant {
    now.wrapping_add(duration_us(value))
}
