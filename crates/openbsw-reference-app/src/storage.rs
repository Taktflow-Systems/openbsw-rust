//! Journaled application storage and blob demonstration.

use bsw_storage::blob::{BlobError, BlobLayout, BlobReader, BlobWriter};
use bsw_storage::block::{BlockId, BlockStore};
use bsw_storage::journal::JournalStore;
use bsw_storage::mem::MemBackend;
use bsw_storage::StorageError;

pub type AppBackend = MemBackend<16_384, 1_024, 4>;
pub type AppStore = JournalStore<AppBackend, 16>;

pub struct PersistentServices {
    store: AppStore,
}

impl PersistentServices {
    pub fn new() -> Result<Self, StorageError> {
        Ok(Self {
            store: JournalStore::mount(AppBackend::new())?,
        })
    }

    pub fn from_backend(backend: AppBackend) -> Result<Self, StorageError> {
        Ok(Self {
            store: JournalStore::mount(backend)?,
        })
    }

    pub fn write_counter(&mut self, block: u16, value: u32) -> Result<(), StorageError> {
        self.store.write(BlockId(block), 1, &value.to_be_bytes())
    }

    pub fn read_counter(&self, block: u16) -> Result<Option<u32>, StorageError> {
        let mut bytes = [0u8; 4];
        match self.store.read(BlockId(block), &mut bytes) {
            Ok(4) => Ok(Some(u32::from_be_bytes(bytes))),
            Ok(_) => Err(StorageError::CorruptData),
            Err(StorageError::UnknownBlock) => Ok(None),
            Err(error) => Err(error),
        }
    }

    pub fn write_blob(&mut self, bytes: &[u8]) -> Result<(), BlobError> {
        let layout = BlobLayout {
            descriptor: BlockId(0x0a10),
            chunk_base: BlockId(0x0a20),
            max_chunks: 8,
            chunk_size: 16,
        };
        let mut writer = BlobWriter::open(&mut self.store, layout, bytes.len() as u32)?;
        for chunk in bytes.chunks(16) {
            writer.write_chunk(chunk)?;
        }
        writer.finalize()
    }

    pub fn read_blob(&self, output: &mut [u8]) -> Result<usize, BlobError> {
        let layout = BlobLayout {
            descriptor: BlockId(0x0a10),
            chunk_base: BlockId(0x0a20),
            max_chunks: 8,
            chunk_size: 16,
        };
        let mut reader = BlobReader::open(&self.store, layout)?;
        if output.len() < reader.total_len() as usize {
            return Err(StorageError::InvalidParameter.into());
        }
        let mut offset = 0;
        for index in 0..reader.chunk_count() {
            let read = reader.read_chunk(index, &mut output[offset..])?;
            offset += read;
        }
        reader.finish()?;
        Ok(offset)
    }

    pub fn restart(self) -> Result<Self, StorageError> {
        Self::from_backend(self.store.into_inner())
    }

    pub fn into_backend(self) -> AppBackend {
        self.store.into_inner()
    }
}
