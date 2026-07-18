//! Diagnostic session, authentication, and security state (E18/E21).

use bsw_time::{Duration, Instant};

use crate::{DiagSession, Nrc};

/// Persistence boundary for session/security metadata.
pub trait StatePersistence {
    /// Restore `(session, failed_attempts)` when valid persisted data exists.
    fn load(&mut self) -> Option<(DiagSession, u8)>;
    /// Atomically persist session and failed-attempt count.
    fn store(&mut self, session: DiagSession, failed_attempts: u8) -> bool;
}

/// Project-owned seed-to-key verification boundary.
pub trait KeyAlgorithm {
    /// Return whether `key` unlocks `level` for the issued `seed`.
    fn verify(&mut self, level: u8, seed: &[u8], key: &[u8]) -> bool;
}

/// Security reset behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SecurityResetPolicy {
    /// Clear unlock state whenever the diagnostic session changes.
    OnSessionChange,
    /// Keep unlock state across sessions; clear only on ECU reset/S3.
    OnReset,
}

/// Bounded security/session state.
pub struct DiagnosticState<P, K> {
    session: DiagSession,
    authenticated: bool,
    unlocked_levels: u32,
    seed: [u8; 16],
    seed_len: usize,
    seed_level: Option<u8>,
    failed_attempts: u8,
    max_attempts: u8,
    delay: Duration,
    delay_until: Instant,
    reset_policy: SecurityResetPolicy,
    persistence: P,
    key_algorithm: K,
}

impl<P: StatePersistence, K: KeyAlgorithm> DiagnosticState<P, K> {
    /// Construct state and restore validated persistence when present.
    pub fn new(
        mut persistence: P,
        key_algorithm: K,
        max_attempts: u8,
        delay: Duration,
        reset_policy: SecurityResetPolicy,
    ) -> Self {
        let (session, failed_attempts) = persistence.load().unwrap_or((DiagSession::Default, 0));
        Self {
            session,
            authenticated: false,
            unlocked_levels: 0,
            seed: [0; 16],
            seed_len: 0,
            seed_level: None,
            failed_attempts,
            max_attempts,
            delay,
            delay_until: Instant::from_nanos(0),
            reset_policy,
            persistence,
            key_algorithm,
        }
    }

    /// Active diagnostic session.
    pub const fn session(&self) -> DiagSession {
        self.session
    }

    /// Change session and persist it.
    pub fn set_session(&mut self, session: DiagSession) -> bool {
        if self.session != session && self.reset_policy == SecurityResetPolicy::OnSessionChange {
            self.clear_security();
        }
        self.session = session;
        self.persistence.store(self.session, self.failed_attempts)
    }

    /// Authentication state used by authentication-aware jobs.
    pub const fn authenticated(&self) -> bool {
        self.authenticated
    }

    /// Set authentication state.
    pub fn set_authenticated(&mut self, authenticated: bool) {
        self.authenticated = authenticated;
    }

    /// Whether one security level is unlocked.
    pub const fn is_unlocked(&self, level: u8) -> bool {
        level < 32 && self.unlocked_levels & (1_u32 << level) != 0
    }

    /// Issue a caller-provided entropy seed for an odd request level.
    pub fn issue_seed(
        &mut self,
        level: u8,
        seed: &[u8],
        now: Instant,
        output: &mut [u8],
    ) -> Result<usize, Nrc> {
        if level == 0 || level & 1 == 0 {
            return Err(Nrc::SubFunctionNotSupported);
        }
        if !now.is_at_or_after(self.delay_until) {
            return Err(Nrc::RequiredTimeDelayNotExpired);
        }
        if seed.is_empty() || seed.len() > self.seed.len() || seed.len() > output.len() {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        self.seed[..seed.len()].copy_from_slice(seed);
        self.seed_len = seed.len();
        self.seed_level = Some(level);
        output[..seed.len()].copy_from_slice(seed);
        Ok(seed.len())
    }

    /// Validate a key for the even level following the last seed request.
    pub fn submit_key(&mut self, level: u8, key: &[u8], now: Instant) -> Result<(), Nrc> {
        if self.seed_level != level.checked_sub(1) {
            return Err(Nrc::RequestSequenceError);
        }
        if !now.is_at_or_after(self.delay_until) {
            return Err(Nrc::RequiredTimeDelayNotExpired);
        }
        if self
            .key_algorithm
            .verify(level, &self.seed[..self.seed_len], key)
        {
            if level < 32 {
                self.unlocked_levels |= 1_u32 << level;
            }
            self.seed_level = None;
            self.failed_attempts = 0;
            self.persistence.store(self.session, self.failed_attempts);
            Ok(())
        } else {
            self.failed_attempts = self.failed_attempts.saturating_add(1);
            self.seed_level = None;
            self.delay_until = now.wrapping_add(self.delay);
            self.persistence.store(self.session, self.failed_attempts);
            if self.failed_attempts >= self.max_attempts {
                Err(Nrc::ExceededNumberOfAttempts)
            } else {
                Err(Nrc::InvalidKey)
            }
        }
    }

    /// Reset to default session and apply the configured security reset.
    pub fn reset(&mut self) {
        self.session = DiagSession::Default;
        self.authenticated = false;
        self.failed_attempts = 0;
        self.delay_until = Instant::from_nanos(0);
        self.clear_security();
        self.persistence.store(self.session, self.failed_attempts);
    }

    /// Failed key attempts retained by persistence.
    pub const fn failed_attempts(&self) -> u8 {
        self.failed_attempts
    }

    /// Security levels as a bit mask for registry access checks.
    pub const fn unlocked_levels(&self) -> u32 {
        self.unlocked_levels
    }

    fn clear_security(&mut self) {
        self.unlocked_levels = 0;
        self.seed.fill(0);
        self.seed_len = 0;
        self.seed_level = None;
    }
}
