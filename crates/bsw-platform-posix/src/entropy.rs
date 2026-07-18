//! Host entropy source without external dependencies.
//!
//! # NOT cryptographic
//!
//! [`PosixEntropy`] is a splitmix64 pseudo-random stream seeded once from
//! OS-provided randomness. It exists so host simulations of the platform
//! contract produce varied, well-distributed bytes; it must **never** be
//! used for key material or any security purpose. Real products supply a
//! hardware RNG behind the same [`Entropy`] trait.
//!
//! Seeding (no new external crates, per the dependency policy):
//! `/dev/urandom` on Unix hosts, mixed with the randomly-keyed
//! `std::collections::hash_map::RandomState` hasher (which std seeds from
//! OS entropy), the wall clock, and the process id as a portable fallback
//! for every other host.

use std::collections::hash_map::RandomState;
use std::hash::{BuildHasher, Hasher};
use std::time::{SystemTime, UNIX_EPOCH};

use bsw_platform::{Entropy, EntropyError};

/// Non-cryptographic host entropy implementing [`Entropy`] (module docs).
#[derive(Debug)]
pub struct PosixEntropy {
    state: u64,
}

impl PosixEntropy {
    /// Create a generator seeded from OS randomness (see module docs).
    #[must_use]
    pub fn new() -> Self {
        Self { state: seed() }
    }

    /// splitmix64 step: full-period, well-distributed, not cryptographic.
    fn next_word(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
}

impl Default for PosixEntropy {
    fn default() -> Self {
        Self::new()
    }
}

impl Entropy for PosixEntropy {
    fn fill(&mut self, buffer: &mut [u8]) -> Result<(), EntropyError> {
        for chunk in buffer.chunks_mut(8) {
            let word = self.next_word().to_le_bytes();
            chunk.copy_from_slice(&word[..chunk.len()]);
        }
        Ok(())
    }
}

/// Read one word of OS randomness from `/dev/urandom` (Unix hosts).
#[cfg(unix)]
fn os_random() -> Option<u64> {
    use std::io::Read;
    let mut file = std::fs::File::open("/dev/urandom").ok()?;
    let mut bytes = [0_u8; 8];
    file.read_exact(&mut bytes).ok()?;
    Some(u64::from_le_bytes(bytes))
}

/// No direct OS randomness path on this host; the `RandomState` mix in
/// [`seed`] carries the OS-seeded randomness instead.
#[cfg(not(unix))]
fn os_random() -> Option<u64> {
    None
}

/// One word from a freshly keyed `RandomState` hasher (OS-seeded by std).
fn hashed_word(salt: u64) -> u64 {
    let mut hasher = RandomState::new().build_hasher();
    hasher.write_u64(salt);
    hasher.write_u64(u64::from(std::process::id()));
    hasher.finish()
}

fn seed() -> u64 {
    let time = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |elapsed| elapsed.as_nanos() as u64);
    let mixed = os_random().unwrap_or(0)
        ^ hashed_word(0x706F_7369_785F_656E)
        ^ hashed_word(0x7472_6F70_795F_7631).rotate_left(31)
        ^ time.rotate_left(13);
    if mixed == 0 {
        // splitmix64 tolerates any seed, but keep the stream away from the
        // low-entropy all-zero starting point.
        0x9E37_79B9_7F4A_7C15
    } else {
        mixed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fill_produces_nonzero_varied_bytes() {
        let mut entropy = PosixEntropy::new();
        let mut first = [0_u8; 32];
        entropy.fill(&mut first).unwrap();
        assert_ne!(first, [0_u8; 32]);
        let mut second = [0_u8; 32];
        entropy.fill(&mut second).unwrap();
        assert_ne!(first, second);
    }

    #[test]
    fn independent_generators_diverge() {
        // Statistical smoke: two OS-seeded generators colliding on a
        // 256-bit stream is negligible.
        let mut a = PosixEntropy::new();
        let mut b = PosixEntropy::new();
        let mut buf_a = [0_u8; 32];
        let mut buf_b = [0_u8; 32];
        a.fill(&mut buf_a).unwrap();
        b.fill(&mut buf_b).unwrap();
        assert_ne!(buf_a, buf_b);
    }

    #[test]
    fn odd_length_buffers_are_filled() {
        let mut entropy = PosixEntropy::new();
        let mut buf = [0_u8; 13];
        entropy.fill(&mut buf).unwrap();
        // All 13 bytes zero after a fill would be a 2^-104 event.
        assert_ne!(buf, [0_u8; 13]);
    }
}
