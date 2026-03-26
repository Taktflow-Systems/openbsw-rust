//! Receive protocol handler — pure state machine for ISO-TP RX sessions.
//!
//! No I/O, no timers, no allocations. The caller drives transitions by calling
//! methods when events occur; the handler returns [`RxTransition`] describing
//! what happened and what the caller must do next (e.g., send a flow control
//! frame, deliver the message to the application layer).
//!
//! # State diagram
//!
//! ```text
//! Allocate ──allocated(ok)──► Send (multi) | Processing (single)
//! Allocate ──allocated(fail)──► Wait (retry) | Fail (max retries)
//! Wait ──expired()──► Allocate (retry)
//! Send ──frame_sent(ok)──► Wait (receive CFs)
//! Send ──frame_sent(fail)──► Fail
//! Wait ──consecutive_frame_received()──► Send | Processing (last CF)
//! Wait ──expired()──► Fail
//! Processing ──processed(ok)──► Done
//! Processing ──processed(fail)──► Fail
//! ```

use crate::constants::ProtocolMessage;

// ── Public types ──────────────────────────────────────────────────────────────

/// RX session states.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RxState {
    /// Waiting for a buffer to be allocated by the upper layer.
    #[default]
    Allocate,
    /// Waiting to send a flow control frame to the transmitter.
    Send,
    /// Waiting to receive the next consecutive frame.
    Wait,
    /// Buffer filled; waiting for the upper layer to process the message.
    Processing,
    /// Message successfully received and processed.
    Done,
    /// Session aborted due to error, timeout, or allocation failure.
    Fail,
}

/// Which timer the RX session is currently waiting on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxTimeout {
    /// No timer active.
    None,
    /// Waiting to receive the next consecutive frame (`N_Cr` timer, ISO §6.7.6).
    Rx,
    /// Waiting before retrying buffer allocation.
    Allocate,
}

/// Result returned by every RX state-machine method.
#[derive(Debug, Clone, Copy)]
pub struct RxTransition {
    /// Whether the handler's state changed.
    pub state_changed: bool,
    /// Caller should now send a flow control frame.
    pub send_flow_control: bool,
    /// If `send_flow_control` is true: `true` = WAIT FC, `false` = CTS FC.
    pub flow_control_wait: bool,
    /// Informational / error message (use [`ProtocolMessage::None`] for normal transitions).
    pub message: ProtocolMessage,
}

// ── RxProtocolHandler ─────────────────────────────────────────────────────────

/// Receive protocol handler — drives a single ISO-TP RX session.
pub struct RxProtocolHandler {
    state: RxState,
    timeout: RxTimeout,
    is_multi_frame: bool,
    /// Total consecutive frames expected (0 for single-frame sessions).
    frame_count: u32,
    /// Number of consecutive frames received so far.
    frame_index: u32,
    /// Block size sent in our CTS FC frames. 0 = unlimited.
    block_size: u8,
    /// Frames received within the current block.
    block_frame_index: u8,
    /// How many allocation retries have been attempted.
    allocate_retry_count: u8,
    max_allocate_retry_count: u8,
}

impl RxProtocolHandler {
    /// Create a handler for a single-frame message.
    ///
    /// Single-frame sessions do not send flow control and go directly to
    /// `Allocate` → `Processing` → `Done`.
    pub fn new_single() -> Self {
        Self {
            state: RxState::Allocate,
            timeout: RxTimeout::Allocate,
            is_multi_frame: false,
            frame_count: 0,
            frame_index: 0,
            block_size: 0,
            block_frame_index: 0,
            allocate_retry_count: 0,
            max_allocate_retry_count: 0, // single-frame never retries
        }
    }

    /// Create a handler for a multi-frame message.
    ///
    /// `frame_count` is the total number of consecutive frames expected
    /// (i.e., total frames minus the first frame).
    /// `block_size` is the block size we advertise in CTS FC frames (0 = unlimited).
    /// `max_allocate_retry` is how many times to retry allocation before failing.
    pub fn new_multi(frame_count: u32, block_size: u8, max_allocate_retry: u8) -> Self {
        Self {
            state: RxState::Allocate,
            timeout: RxTimeout::Allocate,
            is_multi_frame: true,
            frame_count,
            frame_index: 0,
            block_size,
            block_frame_index: 0,
            allocate_retry_count: 0,
            max_allocate_retry_count: max_allocate_retry,
        }
    }

    // ── Accessors ─────────────────────────────────────────────────────────────

    /// Current handler state.
    pub fn state(&self) -> RxState {
        self.state
    }

    /// Which timer the session is currently waiting on.
    pub fn timeout(&self) -> RxTimeout {
        self.timeout
    }

    /// Number of consecutive frames received so far.
    pub fn frame_index(&self) -> u32 {
        self.frame_index
    }

    /// Returns `true` for multi-frame sessions.
    pub fn is_multi_frame(&self) -> bool {
        self.is_multi_frame
    }

    // ── State transitions ─────────────────────────────────────────────────────

    /// Notify the handler that a buffer allocation attempt completed.
    ///
    /// - `success = true`: allocation succeeded.
    ///   - Single-frame → `Processing`.
    ///   - Multi-frame → `Send` (send CTS FC).
    /// - `success = false`: allocation failed.
    ///   - If retries remain → send WAIT FC and wait to retry.
    ///   - Otherwise → `Fail`.
    pub fn allocated(&mut self, success: bool) -> RxTransition {
        if self.state != RxState::Allocate {
            return no_change();
        }

        if success {
            self.allocate_retry_count = 0;
            if self.is_multi_frame {
                self.state = RxState::Send;
                self.timeout = RxTimeout::None;
                RxTransition {
                    state_changed: true,
                    send_flow_control: true,
                    flow_control_wait: false, // CTS
                    message: ProtocolMessage::None,
                }
            } else {
                self.state = RxState::Processing;
                self.timeout = RxTimeout::None;
                RxTransition {
                    state_changed: true,
                    send_flow_control: false,
                    flow_control_wait: false,
                    message: ProtocolMessage::None,
                }
            }
        } else {
            self.allocate_retry_count += 1;
            if self.allocate_retry_count > self.max_allocate_retry_count {
                // Exhausted retries
                self.state = RxState::Fail;
                self.timeout = RxTimeout::None;
                RxTransition {
                    state_changed: true,
                    send_flow_control: false,
                    flow_control_wait: false,
                    message: ProtocolMessage::AllocationFailed,
                }
            } else {
                // Send WAIT FC (if multi-frame) and schedule retry
                self.timeout = RxTimeout::Allocate;
                RxTransition {
                    state_changed: false,
                    send_flow_control: self.is_multi_frame,
                    flow_control_wait: true, // WAIT
                    message: ProtocolMessage::None,
                }
            }
        }
    }

    /// Notify the handler that a consecutive frame arrived.
    ///
    /// Advances the frame index. If:
    /// - This is the last expected CF → `Processing`.
    /// - We've reached the block boundary → `Send` (send another CTS FC).
    /// - Otherwise → stay in `Wait`.
    pub fn consecutive_frame_received(&mut self) -> RxTransition {
        if self.state != RxState::Wait {
            return no_change();
        }

        self.frame_index += 1;
        self.block_frame_index += 1;

        if self.frame_index >= self.frame_count {
            // Last consecutive frame — message complete
            self.state = RxState::Processing;
            self.timeout = RxTimeout::None;
            RxTransition {
                state_changed: true,
                send_flow_control: false,
                flow_control_wait: false,
                message: ProtocolMessage::None,
            }
        } else if self.block_size > 0 && self.block_frame_index >= self.block_size {
            // Block boundary reached — send another CTS FC
            self.block_frame_index = 0;
            self.state = RxState::Send;
            self.timeout = RxTimeout::None;
            RxTransition {
                state_changed: true,
                send_flow_control: true,
                flow_control_wait: false, // CTS
                message: ProtocolMessage::None,
            }
        } else {
            // Continue receiving
            self.timeout = RxTimeout::Rx;
            RxTransition {
                state_changed: false,
                send_flow_control: false,
                flow_control_wait: false,
                message: ProtocolMessage::None,
            }
        }
    }

    /// Notify the handler that the flow control frame was sent (or failed to send).
    ///
    /// - `success = true` → `Wait` (start `N_Cr` timer, wait for next CF).
    /// - `success = false` → `Fail`.
    pub fn frame_sent(&mut self, success: bool) -> RxTransition {
        if self.state != RxState::Send {
            return no_change();
        }

        if success {
            self.state = RxState::Wait;
            self.timeout = RxTimeout::Rx;
            RxTransition {
                state_changed: true,
                send_flow_control: false,
                flow_control_wait: false,
                message: ProtocolMessage::None,
            }
        } else {
            self.state = RxState::Fail;
            self.timeout = RxTimeout::None;
            RxTransition {
                state_changed: true,
                send_flow_control: false,
                flow_control_wait: false,
                message: ProtocolMessage::SendFailed,
            }
        }
    }

    /// Notify the handler that the upper layer finished processing the message.
    ///
    /// - `success = true` → `Done`.
    /// - `success = false` → `Fail`.
    pub fn processed(&mut self, success: bool) -> RxTransition {
        if self.state != RxState::Processing {
            return no_change();
        }

        if success {
            self.state = RxState::Done;
            self.timeout = RxTimeout::None;
            RxTransition {
                state_changed: true,
                send_flow_control: false,
                flow_control_wait: false,
                message: ProtocolMessage::None,
            }
        } else {
            self.state = RxState::Fail;
            self.timeout = RxTimeout::None;
            RxTransition {
                state_changed: true,
                send_flow_control: false,
                flow_control_wait: false,
                message: ProtocolMessage::None,
            }
        }
    }

    /// Notify the handler that the active timer has expired.
    ///
    /// - During `Allocate` wait → retry allocation (back to `Allocate`).
    /// - During `Wait` (`N_Cr` timeout) → `Fail`.
    /// - Any terminal state → no-op.
    pub fn expired(&mut self) -> RxTransition {
        match self.state {
            RxState::Allocate => {
                // Retry allocate — reset to Allocate state for another attempt.
                // (Already in Allocate; just signal the caller to retry.)
                RxTransition {
                    state_changed: false,
                    send_flow_control: false,
                    flow_control_wait: false,
                    message: ProtocolMessage::None,
                }
            }
            RxState::Done | RxState::Fail => no_change(),
            // Wait, Send, Processing — any unexpected/active state → Fail with Timeout
            _ => {
                self.state = RxState::Fail;
                self.timeout = RxTimeout::None;
                RxTransition {
                    state_changed: true,
                    send_flow_control: false,
                    flow_control_wait: false,
                    message: ProtocolMessage::Timeout,
                }
            }
        }
    }
}

// ── Helpers ────────────────────────────────────────────────────────────────────

#[inline]
fn no_change() -> RxTransition {
    RxTransition {
        state_changed: false,
        send_flow_control: false,
        flow_control_wait: false,
        message: ProtocolMessage::None,
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Single frame ──────────────────────────────────────────────────────────

    #[test]
    fn single_frame_allocate_process_done() {
        let mut h = RxProtocolHandler::new_single();
        assert_eq!(h.state(), RxState::Allocate);
        assert!(!h.is_multi_frame());

        let t = h.allocated(true);
        assert!(t.state_changed);
        assert!(!t.send_flow_control);
        assert_eq!(h.state(), RxState::Processing);

        let t = h.processed(true);
        assert!(t.state_changed);
        assert_eq!(h.state(), RxState::Done);
    }

    #[test]
    fn single_frame_no_flow_control() {
        let mut h = RxProtocolHandler::new_single();
        let t = h.allocated(true);
        assert!(!t.send_flow_control);
    }

    #[test]
    fn single_frame_allocation_failure_retries() {
        // Single-frame: max_allocate_retry = 0, so first failure → Fail
        let mut h = RxProtocolHandler::new_single();
        let t = h.allocated(false);
        assert!(t.state_changed);
        assert_eq!(h.state(), RxState::Fail);
        assert_eq!(t.message, ProtocolMessage::AllocationFailed);
    }

    // ── Multi-frame ───────────────────────────────────────────────────────────

    #[test]
    fn multi_frame_allocate_send_fc_wait_receive() {
        // 2 CFs, unlimited block
        let mut h = RxProtocolHandler::new_multi(2, 0, 3);
        assert!(h.is_multi_frame());

        // Allocation → Send CTS FC
        let t = h.allocated(true);
        assert!(t.send_flow_control);
        assert!(!t.flow_control_wait); // CTS, not WAIT
        assert_eq!(h.state(), RxState::Send);

        // FC sent → Wait
        let t = h.frame_sent(true);
        assert!(t.state_changed);
        assert_eq!(h.state(), RxState::Wait);

        // First CF
        let t = h.consecutive_frame_received();
        assert!(!t.state_changed); // still waiting for second
        assert_eq!(h.frame_index(), 1);
    }

    #[test]
    fn multi_frame_complete_sequence() {
        // 3 CFs, unlimited block
        let mut h = RxProtocolHandler::new_multi(3, 0, 3);
        h.allocated(true);
        h.frame_sent(true); // → Wait

        h.consecutive_frame_received(); // frame 1
        h.consecutive_frame_received(); // frame 2

        let t = h.consecutive_frame_received(); // frame 3 (last)
        assert!(t.state_changed);
        assert_eq!(h.state(), RxState::Processing);

        let t = h.processed(true);
        assert_eq!(h.state(), RxState::Done);
        assert!(t.state_changed);
    }

    #[test]
    fn multi_frame_allocation_sends_wait_fc() {
        let mut h = RxProtocolHandler::new_multi(3, 0, 3);
        // First allocation fails
        let t = h.allocated(false);
        assert!(!t.state_changed); // still in Allocate
        assert!(t.send_flow_control);
        assert!(t.flow_control_wait); // WAIT FC
        assert_eq!(h.state(), RxState::Allocate);
    }

    #[test]
    fn block_boundary_sends_fc() {
        // 4 CFs, block_size = 2
        let mut h = RxProtocolHandler::new_multi(4, 2, 3);
        h.allocated(true);
        h.frame_sent(true); // → Wait

        // Receive 2 CFs → block boundary
        h.consecutive_frame_received(); // frame 1 (block_frame_index = 1)
        let t = h.consecutive_frame_received(); // frame 2 → block boundary
        assert!(t.state_changed);
        assert!(t.send_flow_control);
        assert!(!t.flow_control_wait); // CTS
        assert_eq!(h.state(), RxState::Send);
    }

    #[test]
    fn rx_timeout_fails() {
        let mut h = RxProtocolHandler::new_multi(3, 0, 3);
        h.allocated(true);
        h.frame_sent(true); // → Wait

        let t = h.expired();
        assert!(t.state_changed);
        assert_eq!(h.state(), RxState::Fail);
        assert_eq!(t.message, ProtocolMessage::Timeout);
    }

    #[test]
    fn max_allocate_retries_fails() {
        // max 2 retries
        let mut h = RxProtocolHandler::new_multi(3, 0, 2);

        // First fail: retry_count = 1 (≤ max 2)
        let t = h.allocated(false);
        assert!(!t.state_changed);
        assert_eq!(h.state(), RxState::Allocate);

        // Second fail: retry_count = 2 (≤ max 2)
        let t = h.allocated(false);
        assert!(!t.state_changed);
        assert_eq!(h.state(), RxState::Allocate);

        // Third fail: retry_count = 3 (> max 2) → Fail
        let t = h.allocated(false);
        assert!(t.state_changed);
        assert_eq!(h.state(), RxState::Fail);
        assert_eq!(t.message, ProtocolMessage::AllocationFailed);
    }

    #[test]
    fn frame_index_advances_on_cf() {
        let mut h = RxProtocolHandler::new_multi(5, 0, 3);
        h.allocated(true);
        h.frame_sent(true);
        assert_eq!(h.frame_index(), 0);
        h.consecutive_frame_received();
        assert_eq!(h.frame_index(), 1);
        h.consecutive_frame_received();
        assert_eq!(h.frame_index(), 2);
    }

    #[test]
    fn sequence_wraps_at_16() {
        // Ensure receiving 16 CFs works (frame_index wraps is caller concern,
        // handler just counts)
        let mut h = RxProtocolHandler::new_multi(16, 0, 3);
        h.allocated(true);
        h.frame_sent(true);
        for _ in 0..15 {
            let t = h.consecutive_frame_received();
            assert!(!t.state_changed);
        }
        let t = h.consecutive_frame_received(); // 16th = last
        assert!(t.state_changed);
        assert_eq!(h.state(), RxState::Processing);
        assert_eq!(h.frame_index(), 16);
    }

    #[test]
    fn processing_to_done() {
        let mut h = RxProtocolHandler::new_single();
        h.allocated(true);
        assert_eq!(h.state(), RxState::Processing);
        let t = h.processed(true);
        assert_eq!(h.state(), RxState::Done);
        assert!(t.state_changed);
    }

    #[test]
    fn consecutive_frame_completes_message() {
        let mut h = RxProtocolHandler::new_multi(1, 0, 3);
        h.allocated(true);
        h.frame_sent(true);
        let t = h.consecutive_frame_received(); // 1st = last
        assert!(t.state_changed);
        assert_eq!(h.state(), RxState::Processing);
    }

    #[test]
    fn block_frame_counter_resets() {
        // block_size = 2, 4 total CFs
        let mut h = RxProtocolHandler::new_multi(4, 2, 3);
        h.allocated(true);
        h.frame_sent(true);

        // First block: 2 CFs
        h.consecutive_frame_received(); // block_frame_index = 1
        let t = h.consecutive_frame_received(); // block_frame_index = 2 → boundary → Send
        assert_eq!(h.state(), RxState::Send);
        assert!(t.send_flow_control);

        // Send the second CTS FC
        h.frame_sent(true); // → Wait again

        // Second block: 2 more CFs
        h.consecutive_frame_received(); // block_frame_index = 1 (reset to 0 then ++)
        let t = h.consecutive_frame_received(); // frame_index = 4 → Processing
        assert_eq!(h.state(), RxState::Processing);
        assert!(!t.send_flow_control);
    }
}
