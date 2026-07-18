#![no_main]

use bsw_can::{CanFrame, CanId};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 5 {
        return;
    }
    let raw = u32::from_le_bytes(data[..4].try_into().unwrap());
    let id = CanId::id(raw, data[4] & 1 != 0);
    if let Ok(mut frame) = CanFrame::try_with_data(id, &data[5..]) {
        let original = frame.payload().to_vec();
        let _ = frame.try_set_payload(&original);
        assert_eq!(frame.payload(), original);
        assert_eq!(frame.id(), id);
    }
});
