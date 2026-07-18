#![no_main]

use bsw_doip::{DiagnosticMessageHandler, MessageHandler, PoolDiagnosticGateway, ProtocolVersion};
use bsw_transport::pool::{AdmissionPolicy, AllocationRequest, ProviderError};
use bsw_transport::{TransportMessage, TransportResult};
use libfuzzer_sys::fuzz_target;

#[derive(Clone, Copy)]
struct Policy;

impl AdmissionPolicy for Policy {
    fn admit(&self, request: &AllocationRequest<'_>) -> Result<(), ProviderError> {
        if request.source == u16::MAX {
            Err(ProviderError::InvalidSourceAddress)
        } else if request.target != 0x0123 {
            Err(ProviderError::InvalidTargetAddress)
        } else {
            Ok(())
        }
    }
}

fuzz_target!(|data: &[u8]| {
    let mut gateway =
        PoolDiagnosticGateway::<2, 512, _, _>::new(Policy, |_: &TransportMessage<512>| {
            TransportResult::Ok
        });
    let mut handler = DiagnosticMessageHandler::new(
        ProtocolVersion::Iso2012,
        1,
        Some(0x1234),
        0x0123,
        512,
        &mut gateway,
    );
    let payload_type = match data.first().copied().unwrap_or(0) % 3 {
        0 => 0x8001,
        1 => 0x8002,
        _ => 0x8003,
    };
    let payload = data.get(1..).unwrap_or_default();
    let length = payload.len().min(512);
    if handler.header_received(payload_type, length as u32) {
        let mut offset = 0usize;
        while offset < length {
            let width = usize::from(payload[offset]).saturating_add(1).min(32);
            let end = (offset + width).min(length);
            handler.payload_chunk(&payload[offset..end]);
            while handler.take_action().is_some() {}
            while handler.take_received_ack().is_some() {}
            offset = end;
        }
    }
});
