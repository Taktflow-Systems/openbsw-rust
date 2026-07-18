#![no_main]

use bsw_docan::addressing::{
    Addressing, ConnectionInfo, DataLinkAddressPair, TransportAddressPair,
};
use bsw_docan::constants::FlowStatus;
use bsw_docan::transport::{ConnectionConfig, RxSession, TxSession};
use bsw_docan::Parameters;
use bsw_time::Instant;
use libfuzzer_sys::fuzz_target;

fn config(selector: u8) -> ConnectionConfig {
    let addressing = match selector % 3 {
        0 => Addressing::normal(),
        1 => Addressing::extended(selector),
        _ => Addressing::mixed(selector),
    };
    ConnectionConfig {
        connection: ConnectionInfo::new(
            TransportAddressPair::new(0x00f1, 0x0006),
            DataLinkAddressPair::new(0x700, 0x708),
        ),
        addressing,
        frame_size: 8,
        filler_byte: 0xcc,
        block_size: selector % 5,
        parameters: Parameters::default(),
    }
}

fuzz_target!(|data: &[u8]| {
    let selector = data.first().copied().unwrap_or(0);
    let config = config(selector);
    let mut tx: TxSession<128> = TxSession::new(config);
    let mut rx: RxSession<128> = RxSession::new(config);
    let payload = data
        .get(1..65)
        .unwrap_or_else(|| data.get(1..).unwrap_or_default());
    let _ = tx.start(payload);
    for (index, byte) in data.iter().copied().enumerate() {
        let now = Instant::from_nanos((index as u64).wrapping_mul(100_000));
        match byte % 6 {
            0 => {
                let _ = tx.poll(now);
            }
            1 => {
                let _ = tx.confirm(byte & 0x80 == 0, now);
            }
            2 => {
                let status = match (byte / 6) % 3 {
                    0 => FlowStatus::ContinueToSend,
                    1 => FlowStatus::Wait,
                    _ => FlowStatus::Overflow,
                };
                let _ = tx.flow_control(status, byte >> 4, byte, now);
            }
            3 => {
                let frame = data.get(index..).unwrap_or_default();
                let _ = rx.receive(frame, now, byte & 0x80 == 0);
            }
            4 => {
                let _ = rx.poll(now, byte & 0x80 == 0);
            }
            _ => {
                tx.cancel();
                rx.abort();
            }
        }
    }
});
