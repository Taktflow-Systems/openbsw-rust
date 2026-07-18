//! `std` adapter running the platform-independent DoCAN core over
//! [`bsw_can::virtual_bus`].

use bsw_can::can_id::CanId;
use bsw_can::frame::CanFrame;
use bsw_can::transceiver::{CanTransceiver, ErrorCode};
use bsw_can::virtual_bus::VirtualNode;
use bsw_time::Instant;
use bsw_transport::TransportMessage;

use crate::codec::{decode_frame, DecodedFrame};
use crate::transport::{
    ConnectionConfig, FrameAction, ReceiveAction, RxSession, SessionError, TxSession,
};

/// Complete DoCAN endpoint with fixed RX/TX message storage.
pub struct VirtualCanTransport<const PAYLOAD: usize> {
    node: VirtualNode,
    config: ConnectionConfig,
    tx: TxSession<PAYLOAD>,
    rx: RxSession<PAYLOAD>,
}

impl<const PAYLOAD: usize> VirtualCanTransport<PAYLOAD> {
    /// Bind a configured core to an already initialized/open virtual node.
    pub const fn new(node: VirtualNode, config: ConnectionConfig) -> Self {
        Self {
            node,
            config,
            tx: TxSession::new(config),
            rx: RxSession::new(config),
        }
    }

    /// Queue one complete transport payload.
    pub fn send(&mut self, payload: &[u8]) -> Result<(), SessionError> {
        self.tx.start(payload)
    }

    /// Drive receive, timers, flow control, and one due transmit frame.
    pub fn cycle(&mut self, now: Instant) -> Result<(), SessionError> {
        while let Some(frame) = self.node.receive() {
            if frame.id().raw_id() != self.config.connection.data_link.reception_id {
                continue;
            }
            if !self.config.addressing.accepts(frame.payload()) {
                continue;
            }
            let codec = self.config.addressing.codec(self.config.filler_byte);
            if let DecodedFrame::FlowControl(flow) =
                decode_frame(frame.payload(), &codec).map_err(|_| SessionError::MalformedFrame)?
            {
                self.tx
                    .flow_control(flow.status, flow.block_size, flow.separation_time, now)?;
            } else {
                let action = self.rx.receive(frame.payload(), now, true)?;
                self.apply_receive_action(action)?;
            }
        }
        let action = self.rx.poll(now, true)?;
        self.apply_receive_action(action)?;
        if let Some(frame) = self.tx.poll(now)? {
            let success = self.write(&frame) == ErrorCode::Ok;
            self.tx.confirm(success, now)?;
        }
        Ok(())
    }

    /// Take a fully reassembled message.
    pub fn take_received(&mut self) -> Option<TransportMessage<PAYLOAD>> {
        self.rx.take_message()
    }

    /// Current transmitter for deterministic inspection/fault tests.
    pub const fn tx(&self) -> &TxSession<PAYLOAD> {
        &self.tx
    }

    fn apply_receive_action(&mut self, action: ReceiveAction) -> Result<(), SessionError> {
        if let ReceiveAction::FlowControl(frame) = action {
            if self.write(&frame) != ErrorCode::Ok {
                return Err(SessionError::LinkFailure);
            }
        }
        Ok(())
    }

    fn write(&mut self, action: &FrameAction) -> ErrorCode {
        let id = self.config.connection.data_link.transmission_id;
        let frame = CanFrame::try_with_data(CanId::id(id, id > 0x7ff), action.bytes())
            .map_err(|_| ())
            .ok();
        frame
            .as_ref()
            .map_or(ErrorCode::TxFail, |frame| self.node.write(frame))
    }
}
