#![no_main]

use bsw_docan::codec::decode_frame;
use bsw_docan::CodecConfig;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let offset = data.first().copied().unwrap_or(0) as usize % 4;
    let frame = data.get(1..).unwrap_or_default();
    let config = CodecConfig {
        pci_offset: offset,
        filler_byte: 0xCC,
    };
    let _ = decode_frame(frame, &config);
});
