#![no_main]

use bsw_io::MemoryQueue;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mut queue: MemoryQueue<128, 32> = MemoryQueue::new();
    let (mut writer, mut reader) = queue.split();
    let mut offset = 0;
    while offset < data.len() {
        let size = usize::from(data[offset] % 32) + 1;
        offset += 1;
        let end = (offset + size).min(data.len());
        if end == offset {
            break;
        }
        let record = &data[offset..end];
        if let Some(slot) = writer.allocate(record.len()) {
            slot.copy_from_slice(record);
            writer.commit();
            if let Some(read) = reader.peek() {
                assert_eq!(read, record);
                reader.release();
            }
        }
        offset = end;
    }
});
