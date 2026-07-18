//! POSIX file-backed storage backend (std only).
//!
//! [`FileBackend`] persists a storage region in a fixed-size file and
//! simulates flash semantics exactly like [`crate::mem::MemBackend`]:
//! erase sets units to `0xFF`, program only clears bits, and unaligned or
//! out-of-range accesses are rejected. A persisted header records the
//! geometry and is validated on reopen, so a store written by one process
//! lifetime can be mounted by the next.

use std::fs::{File, OpenOptions};
use std::io::{Read as _, Seek, SeekFrom, Write as _};
use std::path::{Path, PathBuf};

use bsw_util::crc::CRC32_ETHERNET;

use crate::backend::{
    check_program_range, check_read_range, geometry_is_valid, StorageBackend, StorageError,
};

/// File header magic: `"BSWSTOR1"`.
const FILE_MAGIC: [u8; 8] = *b"BSWSTOR1";
/// Serialized header size; region data starts at this file offset.
const FILE_HEADER_LEN: usize = 32;

/// Geometry of a [`FileBackend`] region.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FileGeometry {
    /// Total region size in bytes.
    pub region_size: usize,
    /// Bytes per erase unit (power of two dividing `region_size`).
    pub erase_unit: usize,
    /// Program granularity (1, 4, or 8, dividing `erase_unit`).
    pub program_unit: usize,
}

/// File-backed [`StorageBackend`] with persisted geometry.
pub struct FileBackend {
    file: File,
    path: PathBuf,
    geometry: FileGeometry,
}

impl FileBackend {
    /// Create a new region file at `path` (truncating any existing file),
    /// write the geometry header, and fill the region with `0xFF`.
    pub fn create(path: &Path, geometry: FileGeometry) -> Result<Self, StorageError> {
        if !geometry_is_valid(
            geometry.region_size,
            geometry.erase_unit,
            geometry.program_unit,
        ) {
            return Err(StorageError::InvalidParameter);
        }
        let mut file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(true)
            .open(path)
            .map_err(|_| StorageError::Io)?;
        file.write_all(&encode_header(geometry))
            .map_err(|_| StorageError::Io)?;
        let blank = vec![0xFFu8; geometry.erase_unit];
        for _ in 0..geometry.region_size / geometry.erase_unit {
            file.write_all(&blank).map_err(|_| StorageError::Io)?;
        }
        file.flush().map_err(|_| StorageError::Io)?;
        Ok(Self {
            file,
            path: path.to_path_buf(),
            geometry,
        })
    }

    /// Open an existing region file and validate its persisted geometry
    /// header.
    ///
    /// Fails with [`StorageError::CorruptData`] when the header is invalid
    /// and [`StorageError::InvalidParameter`] when the file length does not
    /// match the recorded geometry.
    pub fn open(path: &Path) -> Result<Self, StorageError> {
        let mut file = OpenOptions::new()
            .read(true)
            .write(true)
            .open(path)
            .map_err(|_| StorageError::Io)?;
        let mut header = [0u8; FILE_HEADER_LEN];
        file.read_exact(&mut header).map_err(|_| StorageError::Io)?;
        let geometry = decode_header(&header).ok_or(StorageError::CorruptData)?;
        let expected_len = (FILE_HEADER_LEN + geometry.region_size) as u64;
        let actual_len = file.metadata().map_err(|_| StorageError::Io)?.len();
        if actual_len != expected_len {
            return Err(StorageError::InvalidParameter);
        }
        Ok(Self {
            file,
            path: path.to_path_buf(),
            geometry,
        })
    }

    /// Open `path` when it exists, otherwise create it with `geometry`.
    ///
    /// When the file exists its persisted geometry must equal `geometry`
    /// ([`StorageError::InvalidParameter`] otherwise).
    pub fn open_or_create(path: &Path, geometry: FileGeometry) -> Result<Self, StorageError> {
        if path.exists() {
            let backend = Self::open(path)?;
            if backend.geometry != geometry {
                return Err(StorageError::InvalidParameter);
            }
            Ok(backend)
        } else {
            Self::create(path, geometry)
        }
    }

    /// Path of the backing file.
    pub fn path(&self) -> &Path {
        &self.path
    }

    /// Geometry recorded in the file header.
    pub fn geometry(&self) -> FileGeometry {
        self.geometry
    }

    /// Force file contents to the operating system (`fsync`).
    ///
    /// The hot path only flushes; call this at shutdown points where
    /// durability against OS crashes matters.
    pub fn sync(&mut self) -> Result<(), StorageError> {
        self.file.sync_all().map_err(|_| StorageError::Io)
    }

    fn read_region(&self, offset: usize, buf: &mut [u8]) -> Result<(), StorageError> {
        let mut file = &self.file;
        file.seek(SeekFrom::Start((FILE_HEADER_LEN + offset) as u64))
            .map_err(|_| StorageError::Io)?;
        file.read_exact(buf).map_err(|_| StorageError::Io)
    }

    fn write_region(&mut self, offset: usize, data: &[u8]) -> Result<(), StorageError> {
        self.file
            .seek(SeekFrom::Start((FILE_HEADER_LEN + offset) as u64))
            .map_err(|_| StorageError::Io)?;
        self.file.write_all(data).map_err(|_| StorageError::Io)?;
        self.file.flush().map_err(|_| StorageError::Io)
    }
}

impl StorageBackend for FileBackend {
    fn region_size(&self) -> usize {
        self.geometry.region_size
    }

    fn erase_unit(&self) -> usize {
        self.geometry.erase_unit
    }

    fn program_unit(&self) -> usize {
        self.geometry.program_unit
    }

    fn read(&self, offset: usize, buf: &mut [u8]) -> Result<(), StorageError> {
        check_read_range(self.geometry.region_size, offset, buf.len())?;
        self.read_region(offset, buf)
    }

    fn erase(&mut self, unit_index: usize) -> Result<(), StorageError> {
        let units = self.geometry.region_size / self.geometry.erase_unit;
        if unit_index >= units {
            return Err(StorageError::OutOfRange);
        }
        let blank = vec![0xFFu8; self.geometry.erase_unit];
        self.write_region(unit_index * self.geometry.erase_unit, &blank)
    }

    fn program(&mut self, offset: usize, data: &[u8]) -> Result<(), StorageError> {
        check_program_range(
            self.geometry.region_size,
            self.geometry.program_unit,
            offset,
            data.len(),
        )?;
        let mut current = vec![0u8; data.len()];
        self.read_region(offset, &mut current)?;
        let mut conflict = false;
        for (cell, &byte) in current.iter_mut().zip(data) {
            let programmed = *cell & byte;
            if programmed != byte {
                conflict = true;
            }
            *cell = programmed;
        }
        self.write_region(offset, &current)?;
        if conflict {
            // Same contract as MemBackend: the AND result is stored (the
            // corruption is detectable) and the violation is reported.
            return Err(StorageError::NotErased);
        }
        Ok(())
    }
}

fn encode_header(geometry: FileGeometry) -> [u8; FILE_HEADER_LEN] {
    let mut header = [0u8; FILE_HEADER_LEN];
    header[0..8].copy_from_slice(&FILE_MAGIC);
    header[8..16].copy_from_slice(&(geometry.region_size as u64).to_le_bytes());
    header[16..20].copy_from_slice(&(geometry.erase_unit as u32).to_le_bytes());
    header[20..24].copy_from_slice(&(geometry.program_unit as u32).to_le_bytes());
    let crc = CRC32_ETHERNET.checksum(&header[0..24]);
    header[24..28].copy_from_slice(&crc.to_le_bytes());
    header
}

fn decode_header(header: &[u8; FILE_HEADER_LEN]) -> Option<FileGeometry> {
    if header[0..8] != FILE_MAGIC {
        return None;
    }
    let stored_crc = u32::from_le_bytes(header[24..28].try_into().ok()?);
    if CRC32_ETHERNET.checksum(&header[0..24]) != stored_crc {
        return None;
    }
    let region_size = usize::try_from(u64::from_le_bytes(header[8..16].try_into().ok()?)).ok()?;
    let erase_unit = usize::try_from(u32::from_le_bytes(header[16..20].try_into().ok()?)).ok()?;
    let program_unit = usize::try_from(u32::from_le_bytes(header[20..24].try_into().ok()?)).ok()?;
    if !geometry_is_valid(region_size, erase_unit, program_unit) {
        return None;
    }
    Some(FileGeometry {
        region_size,
        erase_unit,
        program_unit,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_round_trip() {
        let geometry = FileGeometry {
            region_size: 2048,
            erase_unit: 256,
            program_unit: 4,
        };
        assert_eq!(decode_header(&encode_header(geometry)), Some(geometry));
    }

    #[test]
    fn header_rejects_corruption() {
        let geometry = FileGeometry {
            region_size: 2048,
            erase_unit: 256,
            program_unit: 4,
        };
        let mut header = encode_header(geometry);
        header[9] ^= 0x01;
        assert_eq!(decode_header(&header), None);
    }

    #[test]
    fn header_rejects_bad_geometry_even_with_valid_crc() {
        let mut header = [0u8; FILE_HEADER_LEN];
        header[0..8].copy_from_slice(&FILE_MAGIC);
        header[8..16].copy_from_slice(&2048u64.to_le_bytes());
        header[16..20].copy_from_slice(&250u32.to_le_bytes());
        header[20..24].copy_from_slice(&4u32.to_le_bytes());
        let crc = CRC32_ETHERNET.checksum(&header[0..24]);
        header[24..28].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(decode_header(&header), None);
    }
}
