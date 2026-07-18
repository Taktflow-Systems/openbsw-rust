#![no_main]

use bsw_doip::{
    DefaultActivationPolicy, NoMessageHandler, ProtocolVersion, ServerConnection,
    TransportParameters,
};
use bsw_time::Instant;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mut connection = ServerConnection::new(
        ProtocolVersion::Iso2012,
        0x0123,
        TransportParameters {
            max_payload_size: 512,
            ..TransportParameters::default()
        },
    );
    let mut policy = DefaultActivationPolicy;
    let mut handler = NoMessageHandler;
    connection.start(Instant::from_nanos(0), &mut policy);

    let mut offset = 0usize;
    let mut sequence = 0u64;
    while offset < data.len() {
        let width = usize::from(data[offset]).saturating_add(1).min(64);
        let end = (offset + width).min(data.len());
        connection.handle_bytes(
            Instant::from_nanos(sequence.wrapping_mul(1_000_000)),
            &data[offset..end],
            &mut policy,
            &mut handler,
        );
        while connection.take_action().is_some() {}
        offset = end;
        sequence = sequence.wrapping_add(1);
    }
    connection.poll(Instant::from_nanos(sequence.wrapping_mul(1_000_000)));
    while connection.take_action().is_some() {}
});
