//! Deterministic transport callbacks, cancellation, and shutdown (package E02).

use crate::TransportMessage;

/// Final result passed to a processed listener.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProcessingResult {
    /// Processing completed successfully.
    Ok,
    /// Processing timed out.
    Timeout,
    /// Receiver/provider overflowed.
    Overflow,
    /// Sender or lifecycle explicitly aborted the operation.
    Aborted,
    /// Other transport failure.
    Error,
}

/// Immediate receive-listener result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReceiveResult {
    /// Listener accepted the message.
    Ok,
    /// Listener rejected the message.
    Error,
}

/// Completion listener for an asynchronous message.
pub trait ProcessedListener<const PAYLOAD: usize> {
    /// Called once after ownership of `message` returns from the layer.
    fn transport_message_processed(
        &mut self,
        token: SendToken,
        message: &TransportMessage<PAYLOAD>,
        result: ProcessingResult,
    );
}

/// Complete-message receive listener.
pub trait MessageListener<const PAYLOAD: usize> {
    /// Called in receive order; `completion_required` indicates whether the
    /// provider expects a later processed callback.
    fn message_received(
        &mut self,
        source_bus: u8,
        message: &mut TransportMessage<PAYLOAD>,
        completion_required: bool,
    ) -> ReceiveResult;
}

/// Generation-checked send identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SendToken {
    slot: u16,
    generation: u16,
}

impl SendToken {
    /// Queue slot used by the job.
    pub const fn slot(self) -> u16 {
        self.slot
    }
}

/// Submission error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubmitError {
    /// No fixed job slot is available.
    Busy,
    /// Layer is shutting down and rejects new work.
    ShuttingDown,
}

#[derive(Debug, Clone)]
struct Job<const PAYLOAD: usize> {
    token: SendToken,
    message: TransportMessage<PAYLOAD>,
    result: Option<ProcessingResult>,
    sequence: u32,
}

/// Fixed-capacity asynchronous send owner.
///
/// A completion is dispatched exactly once. Ready callbacks are ordered by
/// submission, even if the underlying protocol reports them out of order.
pub struct AsyncTransport<const JOBS: usize, const PAYLOAD: usize> {
    jobs: [Option<Job<PAYLOAD>>; JOBS],
    generations: [u16; JOBS],
    next_sequence: u32,
    shutting_down: bool,
}

impl<const JOBS: usize, const PAYLOAD: usize> AsyncTransport<JOBS, PAYLOAD> {
    /// Create an accepting layer with no outstanding jobs.
    pub fn new() -> Self {
        Self {
            jobs: core::array::from_fn(|_| None),
            generations: [0; JOBS],
            next_sequence: 0,
            shutting_down: false,
        }
    }

    /// Transfer message ownership into the asynchronous layer.
    pub fn submit(&mut self, message: TransportMessage<PAYLOAD>) -> Result<SendToken, SubmitError> {
        if self.shutting_down {
            return Err(SubmitError::ShuttingDown);
        }
        let Some(slot) = self.jobs.iter().position(Option::is_none) else {
            return Err(SubmitError::Busy);
        };
        let token = SendToken {
            slot: slot as u16,
            generation: self.generations[slot],
        };
        self.jobs[slot] = Some(Job {
            token,
            message,
            result: None,
            sequence: self.next_sequence,
        });
        self.next_sequence = self.next_sequence.wrapping_add(1);
        Ok(token)
    }

    /// Mark a job complete. Stale and duplicate reports are ignored.
    pub fn complete(&mut self, token: SendToken, result: ProcessingResult) -> bool {
        let Some(job) = self.job_mut(token) else {
            return false;
        };
        if job.result.is_some() {
            return false;
        }
        job.result = Some(result);
        true
    }

    /// Cancel an outstanding job.
    pub fn cancel(&mut self, token: SendToken) -> bool {
        self.complete(token, ProcessingResult::Aborted)
    }

    /// Reject new jobs and asynchronously abort every outstanding job.
    pub fn shutdown(&mut self) {
        self.shutting_down = true;
        for job in self.jobs.iter_mut().flatten() {
            if job.result.is_none() {
                job.result = Some(ProcessingResult::Aborted);
            }
        }
    }

    /// Return to accepting state once every shutdown callback was dispatched.
    pub fn restart(&mut self) -> bool {
        if self.jobs.iter().any(Option::is_some) {
            return false;
        }
        self.shutting_down = false;
        true
    }

    /// Dispatch the oldest submitted ready job.
    pub fn dispatch_one(&mut self, listener: &mut dyn ProcessedListener<PAYLOAD>) -> bool {
        let candidate = self
            .jobs
            .iter()
            .enumerate()
            .filter_map(|(slot, job)| {
                let job = job.as_ref()?;
                job.result.map(|_| (slot, job.sequence))
            })
            .min_by_key(|(_, sequence)| *sequence);
        let Some((slot, _)) = candidate else {
            return false;
        };
        let Some(job) = self.jobs[slot].take() else {
            return false;
        };
        let Some(result) = job.result else {
            self.jobs[slot] = Some(job);
            return false;
        };
        listener.transport_message_processed(job.token, &job.message, result);
        self.generations[slot] = self.generations[slot].wrapping_add(1);
        true
    }

    /// Outstanding job count, including ready callbacks not yet dispatched.
    pub fn pending(&self) -> usize {
        self.jobs.iter().filter(|job| job.is_some()).count()
    }

    fn job_mut(&mut self, token: SendToken) -> Option<&mut Job<PAYLOAD>> {
        let slot = usize::from(token.slot);
        let job = self.jobs.get_mut(slot)?.as_mut()?;
        (job.token == token).then_some(job)
    }
}

impl<const JOBS: usize, const PAYLOAD: usize> Default for AsyncTransport<JOBS, PAYLOAD> {
    fn default() -> Self {
        Self::new()
    }
}
