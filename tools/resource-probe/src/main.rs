use std::hint::black_box;
use std::mem::size_of;
use std::time::Instant;

use bsw_bsp_stm32::can_health::CanHealth;
use bsw_bsp_stm32::can_isr::InterruptQueue;
use bsw_bsp_stm32::timer::CycleAccumulator;
use bsw_can::CanFrame;
use bsw_console::Console;
use bsw_docan::transport::{RxSession, TxSession};
use bsw_doip::DiscoveryEntity;
use bsw_estd::{BoundedString, FixedVec, ObjectPool};
use bsw_io::MemoryQueue;
use bsw_logger::buffer::BufferedLogger;
use bsw_logger::{bsw_log, ComponentId, Level, LevelFilter, Log};
use bsw_middleware::message::Message as MiddlewareMessage;
use bsw_time::{FakeClock, TimerQueue};
use bsw_transport::pool::MessagePool;
use bsw_uds::connection::ConnectionPool;
use bsw_util::crc::CRC32_ETHERNET;

fn main() {
    const BYTES: usize = 1024 * 1024;
    const ITERATIONS: usize = 16;
    let data = vec![0xA5_u8; BYTES];
    let started = Instant::now();
    let mut checksum = 0_u32;
    for _ in 0..ITERATIONS {
        checksum ^= CRC32_ETHERNET.checksum(black_box(&data));
    }
    black_box(checksum);
    let crc_ns = started.elapsed().as_nanos().max(1);
    let crc_bytes_per_second = (BYTES as u128 * ITERATIONS as u128 * 1_000_000_000) / crc_ns;

    let started = Instant::now();
    let mut vector: FixedVec<u32, 256> = FixedVec::new();
    for round in 0..2_000_u32 {
        for value in 0..256_u32 {
            black_box(vector.push(value ^ round)).unwrap();
        }
        while let Some(value) = vector.pop() {
            black_box(value);
        }
    }
    let vector_ops_per_second =
        (1_024_000_u128 * 1_000_000_000) / started.elapsed().as_nanos().max(1);

    // A statement above the compile-time maximum must fold to nothing:
    // measure that the disabled path is effectively free (package C13).
    struct CountingLog(u64);
    impl Log for CountingLog {
        fn enabled(&self, _: ComponentId, _: Level) -> bool {
            true
        }
        fn log(&mut self, _: ComponentId, _: Level, _: core::fmt::Arguments<'_>) {
            self.0 += 1;
        }
    }
    const DISABLED_CALLS: u64 = 50_000_000;
    let mut logger = CountingLog(0);
    let started = Instant::now();
    for index in 0..DISABLED_CALLS {
        bsw_log!(
            max: LevelFilter::Off,
            &mut logger,
            ComponentId::new(0),
            Level::Critical,
            "value {}",
            black_box(index)
        );
    }
    black_box(&logger.0);
    assert_eq!(logger.0, 0, "disabled logging must never reach the backend");
    let disabled_log_ops_per_second =
        (u128::from(DISABLED_CALLS) * 1_000_000_000) / started.elapsed().as_nanos().max(1);

    println!(
        concat!(
            "{{\"buffers\":{{",
            "\"can_frame\":{},\"memory_queue_1024_64\":{},",
            "\"fixed_vec_u8_4096\":{},\"object_pool_64x16\":{},",
            "\"bounded_string_128\":{},\"timer_queue_32\":{},",
            "\"buffered_logger_4x8x64\":{},\"console_4x64\":{},",
            "\"transport_pool_4x4095\":{},\"docan_tx_4095\":{},",
            "\"docan_rx_4095\":{},\"uds_connections_4x4095\":{},",
            "\"middleware_message_256\":{},\"doip_discovery_entity\":{},",
            "\"stm32_can_interrupt_queue_32\":{},",
            "\"stm32_cycle_accumulator\":{},\"stm32_can_health\":{}",
            "}},\"benchmarks\":{{\"crc32_bytes_per_second\":{},",
            "\"fixed_vec_ops_per_second\":{},",
            "\"disabled_log_ops_per_second\":{}}}}}"
        ),
        size_of::<CanFrame>(),
        size_of::<MemoryQueue<1024, 64>>(),
        size_of::<FixedVec<u8, 4096>>(),
        size_of::<ObjectPool<[u8; 64], 16, 2>>(),
        size_of::<BoundedString<128>>(),
        size_of::<TimerQueue<32>>(),
        size_of::<BufferedLogger<FakeClock, 4, 8, 64>>(),
        size_of::<Console<'static, 4, 64>>(),
        size_of::<MessagePool<4, 4095>>(),
        size_of::<TxSession<4095>>(),
        size_of::<RxSession<4095>>(),
        size_of::<ConnectionPool<4, 4095, 4095>>(),
        size_of::<MiddlewareMessage<256>>(),
        size_of::<DiscoveryEntity>(),
        size_of::<InterruptQueue<32>>(),
        size_of::<CycleAccumulator>(),
        size_of::<CanHealth>(),
        crc_bytes_per_second,
        vector_ops_per_second,
        disabled_log_ops_per_second,
    );
}
