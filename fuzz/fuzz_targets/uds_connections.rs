#![no_main]

use bsw_uds::{
    connection::{ConnectionPool, ConnectionRequest, Direction},
    dispatcher::RequestAddressing,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mut pool = ConnectionPool::<2, 64, 64>::new();
    let addressing = if data.first().copied().unwrap_or(0) & 1 == 0 {
        RequestAddressing::Physical
    } else {
        RequestAddressing::Functional
    };
    if let Ok(handle) = pool.acquire(ConnectionRequest {
        direction: Direction::Incoming,
        addressing,
        source_bus: data.get(1).copied().unwrap_or(0),
        source: 0x0e00,
        target: 0x0e80,
        payload: data.get(2..).unwrap_or_default(),
        notify_processed: true,
    }) {
        for operation in data.iter().copied().skip(2) {
            match operation % 4 {
                0 => {
                    let _ = pool.advance(handle);
                }
                1 => {
                    let _ = pool.cancel(handle);
                }
                2 => {
                    let _ = pool.get(handle);
                }
                _ => {
                    let _ = pool.get_mut(handle);
                }
            }
        }
        let _ = pool.release(handle);
        let _ = pool.release(handle);
    }
});
