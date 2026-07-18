//! Host identity and platform description.
//!
//! [`PosixUniqueId`] derives a host-stable identifier from the machine's
//! hostname. The raw hostname is **never** embedded in the id: hostnames
//! are frequently personal or organizational (user-PC names, site naming
//! schemes), and the unique id flows into diagnostics payloads and logs
//! that may leave the machine. Hashing (two independent FNV-1a lanes)
//! keeps the id stable per host without carrying the name itself; it is an
//! anti-embedding measure, not cryptographic anonymization.

use bsw_platform::{PlatformInfo, UniqueId, UNIQUE_ID_MAX};

const FNV_OFFSET_BASIS: u64 = 0xCBF2_9CE4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01B3;
/// Domain tag (ASCII "bswposix") separating the second FNV lane.
const LANE_TAG: u64 = 0x6273_7770_6F73_6978;

fn fnv1a_64(seed: u64, bytes: &[u8]) -> u64 {
    let mut hash = seed;
    for &byte in bytes {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(FNV_PRIME);
    }
    hash
}

fn derive_id(name: &str) -> [u8; UNIQUE_ID_MAX] {
    let low = fnv1a_64(FNV_OFFSET_BASIS, name.as_bytes());
    let high = fnv1a_64(FNV_OFFSET_BASIS ^ LANE_TAG, name.as_bytes());
    let mut id = [0_u8; UNIQUE_ID_MAX];
    id[..8].copy_from_slice(&low.to_le_bytes());
    id[8..].copy_from_slice(&high.to_le_bytes());
    id
}

/// Best-effort hostname via portable std facilities only (no new deps):
/// `COMPUTERNAME` (Windows), `HOSTNAME` (some Unix shells), then
/// `/etc/hostname` (Linux), then a fixed fallback. The fallback means two
/// unnamed hosts share an id — acceptable for a host-simulation platform.
fn host_name() -> String {
    for var in ["COMPUTERNAME", "HOSTNAME"] {
        if let Ok(name) = std::env::var(var) {
            let trimmed = name.trim();
            if !trimmed.is_empty() {
                return trimmed.to_string();
            }
        }
    }
    #[cfg(unix)]
    if let Ok(contents) = std::fs::read_to_string("/etc/hostname") {
        let trimmed = contents.trim();
        if !trimmed.is_empty() {
            return trimmed.to_string();
        }
    }
    String::from("unknown-host")
}

/// Host-stable unique identity implementing [`UniqueId`] (module docs).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PosixUniqueId {
    id: [u8; UNIQUE_ID_MAX],
}

impl PosixUniqueId {
    /// Derive the identity of the current host from its hostname.
    #[must_use]
    pub fn new() -> Self {
        Self::from_name(&host_name())
    }

    /// Derive an identity from an explicit name (deterministic; used by
    /// tests and by containerized deployments that inject a logical name).
    #[must_use]
    pub fn from_name(name: &str) -> Self {
        Self {
            id: derive_id(name),
        }
    }
}

impl Default for PosixUniqueId {
    fn default() -> Self {
        Self::new()
    }
}

impl UniqueId for PosixUniqueId {
    fn unique_id(&self, out: &mut [u8; UNIQUE_ID_MAX]) -> usize {
        out.copy_from_slice(&self.id);
        UNIQUE_ID_MAX
    }
}

/// Static description of the POSIX host platform.
///
/// `cpu_frequency_hz` is the documented constant `0`: the [`PlatformInfo`]
/// contract defines 0 as "meaningless on a host", and std offers no
/// portable way to query the core clock without new dependencies.
#[derive(Debug, Clone, Copy, Default)]
pub struct PosixPlatformInfo;

impl PosixPlatformInfo {
    /// Create the platform description.
    #[must_use]
    pub const fn new() -> Self {
        Self
    }
}

impl PlatformInfo for PosixPlatformInfo {
    fn platform_name(&self) -> &'static str {
        "posix"
    }

    fn cpu_frequency_hz(&self) -> u32 {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn id_is_deterministic_per_name_and_distinct_across_names() {
        let a1 = PosixUniqueId::from_name("host-a");
        let a2 = PosixUniqueId::from_name("host-a");
        let b = PosixUniqueId::from_name("host-b");
        assert_eq!(a1, a2);
        assert_ne!(a1, b);
    }

    #[test]
    fn id_does_not_embed_the_raw_name() {
        let name = "hostname-that-must-stay-private";
        let identity = PosixUniqueId::from_name(name);
        let mut out = [0_u8; UNIQUE_ID_MAX];
        assert_eq!(identity.unique_id(&mut out), UNIQUE_ID_MAX);
        assert_ne!(&out[..], &name.as_bytes()[..UNIQUE_ID_MAX]);
    }

    #[test]
    fn current_host_id_is_stable_within_a_process() {
        let first = PosixUniqueId::new();
        let second = PosixUniqueId::new();
        assert_eq!(first, second);
    }

    #[test]
    fn platform_info_reports_posix_host_constants() {
        let info = PosixPlatformInfo::new();
        assert_eq!(info.platform_name(), "posix");
        assert_eq!(info.cpu_frequency_hz(), 0);
    }
}
