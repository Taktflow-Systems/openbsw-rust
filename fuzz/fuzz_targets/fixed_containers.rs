#![no_main]

use bsw_estd::FixedVec;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mut values: FixedVec<Vec<u8>, 8> = FixedVec::new();
    for chunk in data.chunks(3) {
        match chunk.first().copied().unwrap_or_default() % 4 {
            0 => {
                let _ = values.push(chunk.to_vec());
            }
            1 if !values.is_empty() => {
                let index = usize::from(chunk.get(1).copied().unwrap_or_default()) % values.len();
                let _ = values.remove(index);
            }
            2 => {
                let _ = values.pop();
            }
            _ => values.clear(),
        }
        assert!(values.len() <= values.capacity());
    }
});
