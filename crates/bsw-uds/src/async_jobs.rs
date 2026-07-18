//! Bounded asynchronous diagnostic job execution (E24).

use crate::Nrc;

/// Stable generation-checked asynchronous job token.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct AsyncToken {
    slot: u16,
    generation: u16,
}

/// Result of starting or resuming an asynchronous job.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AsyncResult {
    Pending,
    Positive(usize),
    Negative(Nrc),
}

/// Asynchronous application job. State remains with the application object.
pub trait AsyncJob {
    /// Start processing a copied request.
    fn start(&mut self, request: &[u8], authenticated: bool, response: &mut [u8]) -> AsyncResult;
    /// Resume after an application event or scheduler tick.
    fn resume(&mut self, authenticated: bool, response: &mut [u8]) -> AsyncResult;
    /// Cancel in-flight application work.
    fn cancel(&mut self) {}
}

/// Completion/cancellation notification returned exactly once.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AsyncCompletion {
    pub token: AsyncToken,
    pub result: AsyncResult,
}

struct Slot<const REQUEST: usize> {
    request: [u8; REQUEST],
    request_len: usize,
    active: bool,
}

/// Fixed-capacity async execution table.
pub struct AsyncExecutor<const COUNT: usize, const REQUEST: usize> {
    slots: [Slot<REQUEST>; COUNT],
    generations: [u16; COUNT],
}

impl<const COUNT: usize, const REQUEST: usize> AsyncExecutor<COUNT, REQUEST> {
    /// Create an empty executor.
    pub fn new() -> Self {
        Self {
            slots: core::array::from_fn(|_| Slot {
                request: [0; REQUEST],
                request_len: 0,
                active: false,
            }),
            generations: [0; COUNT],
        }
    }

    /// Copy a request into a bounded slot and start the job.
    pub fn submit(
        &mut self,
        request: &[u8],
        authenticated: bool,
        job: &mut dyn AsyncJob,
        response: &mut [u8],
    ) -> Result<(AsyncToken, AsyncResult), Nrc> {
        if request.len() > REQUEST {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        let Some(index) = self.slots.iter().position(|slot| !slot.active) else {
            return Err(Nrc::BusyRepeatRequest);
        };
        let slot = &mut self.slots[index];
        slot.request[..request.len()].copy_from_slice(request);
        slot.request_len = request.len();
        slot.active = true;
        let token = AsyncToken {
            slot: index as u16,
            generation: self.generations[index],
        };
        let result = job.start(&slot.request[..slot.request_len], authenticated, response);
        if !matches!(result, AsyncResult::Pending) {
            self.finish(index);
        }
        Ok((token, result))
    }

    /// Resume one pending job.
    pub fn resume(
        &mut self,
        token: AsyncToken,
        authenticated: bool,
        job: &mut dyn AsyncJob,
        response: &mut [u8],
    ) -> Result<AsyncCompletion, Nrc> {
        let index = self.validate(token)?;
        let result = job.resume(authenticated, response);
        if !matches!(result, AsyncResult::Pending) {
            self.finish(index);
        }
        Ok(AsyncCompletion { token, result })
    }

    /// Cancel and release one job.
    pub fn cancel(
        &mut self,
        token: AsyncToken,
        job: &mut dyn AsyncJob,
    ) -> Result<AsyncCompletion, Nrc> {
        let index = self.validate(token)?;
        job.cancel();
        self.finish(index);
        Ok(AsyncCompletion {
            token,
            result: AsyncResult::Negative(Nrc::GeneralReject),
        })
    }

    /// Number of active jobs.
    pub fn in_use(&self) -> usize {
        self.slots.iter().filter(|slot| slot.active).count()
    }

    fn validate(&self, token: AsyncToken) -> Result<usize, Nrc> {
        let index = usize::from(token.slot);
        if index >= COUNT
            || self.generations[index] != token.generation
            || !self.slots[index].active
        {
            Err(Nrc::RequestSequenceError)
        } else {
            Ok(index)
        }
    }

    fn finish(&mut self, index: usize) {
        self.slots[index].request.fill(0);
        self.slots[index].request_len = 0;
        self.slots[index].active = false;
        self.generations[index] = self.generations[index].wrapping_add(1);
    }
}

impl<const COUNT: usize, const REQUEST: usize> Default for AsyncExecutor<COUNT, REQUEST> {
    fn default() -> Self {
        Self::new()
    }
}
