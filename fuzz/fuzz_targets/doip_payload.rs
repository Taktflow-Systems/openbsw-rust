#![no_main]

use bsw_doip::Packet;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if let Ok(packet) = Packet::parse(data) {
        let mut output = [0_u8; 4096];
        let _ = packet.encode(&mut output);
    }
});
