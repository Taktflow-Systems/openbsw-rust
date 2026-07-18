#![no_main]

use bsw_doip::DoIpHeader;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if let Ok(raw) = <&[u8; 8]>::try_from(data) {
        if let Ok(header) = DoIpHeader::parse(raw) {
            let encoded = header.encode();
            assert_eq!(DoIpHeader::parse(&encoded), Ok(header));
        }
    }
});
