//! Transmit protocol handler — pure state machine for ISO-TP TX sessions.
//!
//! No I/O, no timers, no allocations. The caller drives transitions by calling
//! methods when events occur; the handler returns [`TxTransition`] describing
//! what happened and what the caller must do next.
//!
//! # State diagram
//!
//! ```text
//!  Initialized ──start()──► Send
//!      Send ──frame_sending()──► Send (or Wait if block end)
//!      Send ──frames_sent() ──► Success  (if last frame confirmed)
//!      Wait ──handle_flow_control(CTS)──► Send
//!      Wait ──handle_flow_control(Wait)──► Wait  (or Fail if wait count exceeded)
//!      Wait ──handle_flow_control(Overflow)──► Fail
//!      * ──expired()──► Fail
//! ```

use crate::constants::{FlowStatus, ProtocolMessage};

// ── Public types ──────────────────────────────────────────────────────────────

/// TX session states.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TxState {
    /// Handler created but not yet started.
    #[default]
    Initialized,
    /// Actively sending consecutive frames.
    Send,
    /// Waiting for a flow control frame from the receiver.
    Wait,
    /// All frames successfully sent and acknowledged.
    Success,
    /// Session aborted due to error or timeout.
    Fail,
}

/// Which timer the TX session is currently waiting on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxTimeout {
    /// No timer active.
    None,
    /// Waiting for the CAN driver's TX-done callback.
    TxCallback,
    /// Waiting for a flow control frame.
    FlowControl,
    /// Honouring the `STmin` separation time between consecutive frames.
    SeparationTime,
}

/// Side-effects requested by a state transition.
#[derive(Debug, Clone, Copy, Default)]
pub struct TxActions {
    /// Caller should store the separation time from the received FC for future CF spacing.
    pub store_separation_time: bool,
    /// Caller should cancel any pending CAN TX that was queued but not yet sent.
    pub cancel_pending_send: bool,
}

/// Result returned by every TX state-machine method.
#[derive(Debug, Clone, Copy)]
pub struct TxTransition {
    /// Whether the handler's state changed.
    pub state_changed: bool,
    /// Side-effects the caller must carry out.
    pub actions: TxActions,
    /// Informational / error message (use [`ProtocolMessage::None`] for normal transitions).
    pub message: ProtocolMessage,
}

// ── Internal types ─────────────────────────────────────────────────────────────

/// Tracks expectation of flow control in the TX session.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FlowControlState {
    /// No FC is pending/expected.
    Unexpected,
    /// Waiting for the first FC after the first frame.
    Expected,
    /// Received CTS — send the next block.
    ReceivedCts,
    /// Received WAIT — hold and re-wait.
    ReceivedWait,
}

// ── TxProtocolHandler ─────────────────────────────────────────────────────────

/// Transmit protocol handler — drives a single ISO-TP TX session.
///
/// Instantiate with [`TxProtocolHandler::new`], call [`start`][Self::start],
/// then drive through frame-sending → flow-control cycles until
/// [`is_complete`][Self::is_complete] returns `true`.
pub struct TxProtocolHandler {
    state: TxState,
    timeout: TxTimeout,
    flow_control_state: FlowControlState,
    /// Total frames needed for this message (1 for SF, ≥2 for multi-frame).
    frame_count: u32,
    /// Index of the *next* frame to send (0-based).
    frame_index: u32,
    /// The `frame_index` value at which the current FC block ends.
    /// When `frame_index == block_end`, we must wait for another FC.
    block_end: u32,
    /// Number of consecutive WAIT FCs received so far.
    wait_count: u8,
    max_wait_count: u8,
}

impl TxProtocolHandler {
    /// Create a new handler for a message that requires `frame_count` frames.
    ///
    /// `max_wait_count` is the maximum number of consecutive WAIT flow controls
    /// allowed before the session is aborted (ISO §6.7.7).
    pub fn new(frame_count: u32, max_wait_count: u8) -> Self {
        Self {
            state: TxState::Initialized,
            timeout: TxTimeout::None,
            flow_control_state: FlowControlState::Unexpected,
            frame_count,
            frame_index: 0,
            block_end: 0,
            wait_count: 0,
            max_wait_count,
        }
    }

    // ── Accessors ─────────────────────────────────────────────────────────────

    /// Current handler state.
    pub fn state(&self) -> TxState {
        self.state
    }

    /// Which timer the session is currently waiting on.
    pub fn timeout(&self) -> TxTimeout {
        self.timeout
    }

    /// Index of the next frame to be sent (0-based).
    pub fn frame_index(&self) -> u32 {
        self.frame_index
    }

    /// Returns `true` when the session has reached [`TxState::Success`] or [`TxState::Fail`].
    pub fn is_complete(&self) -> bool {
        matches!(self.state, TxState::Success | TxState::Fail)
    }

    // ── State transitions ─────────────────────────────────────────────────────

    /// Start the session: move from `Initialized` → `Send`.
    ///
    /// For multi-frame messages, the first flow control is expected after the
    /// first frame is acknowledged.
    pub fn start(&mut self) -> TxTransition {
        if self.state != TxState::Initialized {
            return no_change();
        }
        self.state = TxState::Send;
        // For multi-frame messages, we must receive a FC after the first frame.
        if self.frame_count > 1 {
            self.flow_control_state = FlowControlState::Expected;
        }
        self.timeout = TxTimeout::TxCallback;
        TxTransition {
            state_changed: true,
            actions: TxActions::default(),
            message: ProtocolMessage::None,
        }
    }

    /// Notify the handler that a frame has been queued to the CAN driver.
    ///
    /// Advances the internal frame index. The caller should now wait for
    /// [`frames_sent`][Self::frames_sent] (TX callback from driver).
    pub fn frame_sending(&mut self) -> TxTransition {
        if self.state != TxState::Send {
            return no_change();
        }
        self.frame_index += 1;
        self.timeout = TxTimeout::TxCallback;
        TxTransition {
            state_changed: false,
            actions: TxActions::default(),
            message: ProtocolMessage::None,
        }
    }

    /// Notify the handler that the CAN driver confirmed the frame was sent.
    ///
    /// - If this was the last frame → `Success`.
    /// - If we've reached the block boundary → `Wait` (need FC).
    /// - Otherwise stay in `Send`.
    pub fn frames_sent(&mut self) -> TxTransition {
        if self.state != TxState::Send {
            return no_change();
        }

        if self.frame_index >= self.frame_count {
            // All frames sent
            self.state = TxState::Success;
            self.timeout = TxTimeout::None;
            return TxTransition {
                state_changed: true,
                actions: TxActions::default(),
                message: ProtocolMessage::None,
            };
        }

        // Check whether we need to wait for a flow control (block end or first FC)
        let need_fc = self.flow_control_state == FlowControlState::Expected
            || (self.block_end > 0 && self.frame_index >= self.block_end);

        if need_fc {
            self.state = TxState::Wait;
            self.timeout = TxTimeout::FlowControl;
            TxTransition {
                state_changed: true,
                actions: TxActions::default(),
                message: ProtocolMessage::None,
            }
        } else {
            // Continue sending — honour STmin by waiting SeparationTime
            self.timeout = TxTimeout::SeparationTime;
            TxTransition {
                state_changed: false,
                actions: TxActions::default(),
                message: ProtocolMessage::None,
            }
        }
    }

    /// Handle an incoming flow control frame.
    ///
    /// Only valid in `Wait` state. Transitions:
    /// - `CTS` → `Send`, sets new block boundary.
    /// - `Wait` → stays in `Wait` (or `Fail` if wait count exceeded).
    /// - `Overflow` → `Fail`.
    pub fn handle_flow_control(
        &mut self,
        status: FlowStatus,
        block_size: u8,
    ) -> TxTransition {
        if self.state != TxState::Wait {
            return TxTransition {
                state_changed: false,
                actions: TxActions::default(),
                message: ProtocolMessage::InvalidFlowControl,
            };
        }

        match status {
            FlowStatus::ContinueToSend => {
                self.flow_control_state = FlowControlState::ReceivedCts;
                self.wait_count = 0;

                // Compute new block boundary
                if block_size == 0 {
                    // Unlimited — never need to wait mid-message
                    self.block_end = 0; // sentinel: 0 means no block limit
                } else {
                    self.block_end = self.frame_index + u32::from(block_size);
                }

                self.state = TxState::Send;
                self.timeout = TxTimeout::SeparationTime;
                TxTransition {
                    state_changed: true,
                    actions: TxActions {
                        store_separation_time: true,
                        cancel_pending_send: false,
                    },
                    message: ProtocolMessage::None,
                }
            }
            FlowStatus::Wait => {
                self.flow_control_state = FlowControlState::ReceivedWait;
                self.wait_count += 1;
                if self.wait_count >= self.max_wait_count {
                    self.state = TxState::Fail;
                    self.timeout = TxTimeout::None;
                    TxTransition {
                        state_changed: true,
                        actions: TxActions {
                            store_separation_time: false,
                            cancel_pending_send: true,
                        },
                        message: ProtocolMessage::WaitCountExceeded,
                    }
                } else {
                    // Stay in Wait, restart FC timer
                    self.timeout = TxTimeout::FlowControl;
                    TxTransition {
                        state_changed: false,
                        actions: TxActions::default(),
                        message: ProtocolMessage::None,
                    }
                }
            }
            FlowStatus::Overflow => {
                self.state = TxState::Fail;
                self.timeout = TxTimeout::None;
                TxTransition {
                    state_changed: true,
                    actions: TxActions {
                        store_separation_time: false,
                        cancel_pending_send: true,
                    },
                    message: ProtocolMessage::Overflow,
                }
            }
        }
    }

    /// Notify the handler that the active timer has expired.
    ///
    /// Always transitions to `Fail`.
    pub fn expired(&mut self) -> TxTransition {
        if self.is_complete() {
            return no_change();
        }
        self.state = TxState::Fail;
        self.timeout = TxTimeout::None;
        TxTransition {
            state_changed: true,
            actions: TxActions {
                store_separation_time: false,
                cancel_pending_send: true,
            },
            message: ProtocolMessage::Timeout,
        }
    }
}

// ── Helpers ────────────────────────────────────────────────────────────────────

#[inline]
fn no_change() -> TxTransition {
    TxTransition {
        state_changed: false,
        actions: TxActions::default(),
        message: ProtocolMessage::None,
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_handler_is_initialized() {
        let h = TxProtocolHandler::new(1, 10);
        assert_eq!(h.state(), TxState::Initialized);
        assert!(!h.is_complete());
        assert_eq!(h.frame_index(), 0);
    }

    #[test]
    fn start_moves_to_send() {
        let mut h = TxProtocolHandler::new(1, 10);
        let t = h.start();
        assert!(t.state_changed);
        assert_eq!(h.state(), TxState::Send);
        assert_eq!(h.timeout(), TxTimeout::TxCallback);
    }

    #[test]
    fn single_frame_send_wait_success() {
        // Single frame: frame_count = 1
        let mut h = TxProtocolHandler::new(1, 10);
        h.start();
        // Notify queued
        let t = h.frame_sending();
        assert!(!t.state_changed);
        assert_eq!(h.frame_index(), 1);
        // Notify confirmed
        let t = h.frames_sent();
        assert!(t.state_changed);
        assert_eq!(h.state(), TxState::Success);
        assert!(h.is_complete());
    }

    #[test]
    fn multi_frame_flow_control() {
        // 3-frame message: FF + 2 CFs
        let mut h = TxProtocolHandler::new(3, 10);
        h.start();

        // Send FF
        h.frame_sending(); // index → 1
        let t = h.frames_sent();
        // After FF, should wait for FC
        assert!(t.state_changed);
        assert_eq!(h.state(), TxState::Wait);
    }

    #[test]
    fn flow_control_cts_resumes_send() {
        let mut h = TxProtocolHandler::new(3, 10);
        h.start();
        h.frame_sending(); // FF queued, index → 1
        h.frames_sent();   // → Wait

        let t = h.handle_flow_control(FlowStatus::ContinueToSend, 0);
        assert!(t.state_changed);
        assert_eq!(h.state(), TxState::Send);
        assert!(t.actions.store_separation_time);
    }

    #[test]
    fn flow_control_wait_increments_counter() {
        let mut h = TxProtocolHandler::new(3, 10);
        h.start();
        h.frame_sending();
        h.frames_sent(); // → Wait

        let t = h.handle_flow_control(FlowStatus::Wait, 0);
        assert!(!t.state_changed);
        assert_eq!(h.state(), TxState::Wait);
        // Internal wait_count = 1 (not directly accessible — test indirectly)
    }

    #[test]
    fn flow_control_overflow_fails() {
        let mut h = TxProtocolHandler::new(3, 10);
        h.start();
        h.frame_sending();
        h.frames_sent(); // → Wait

        let t = h.handle_flow_control(FlowStatus::Overflow, 0);
        assert!(t.state_changed);
        assert_eq!(h.state(), TxState::Fail);
        assert_eq!(t.message, ProtocolMessage::Overflow);
        assert!(t.actions.cancel_pending_send);
    }

    #[test]
    fn wait_count_exceeded_fails() {
        let mut h = TxProtocolHandler::new(3, 3); // max 3 waits
        h.start();
        h.frame_sending();
        h.frames_sent(); // → Wait

        // 2 waits: still alive
        h.handle_flow_control(FlowStatus::Wait, 0);
        h.handle_flow_control(FlowStatus::Wait, 0);
        let t = h.handle_flow_control(FlowStatus::Wait, 0); // 3rd → Fail
        assert!(t.state_changed);
        assert_eq!(h.state(), TxState::Fail);
        assert_eq!(t.message, ProtocolMessage::WaitCountExceeded);
    }

    #[test]
    fn tx_callback_timeout_fails() {
        let mut h = TxProtocolHandler::new(1, 10);
        h.start();
        let t = h.expired();
        assert!(t.state_changed);
        assert_eq!(h.state(), TxState::Fail);
        assert_eq!(t.message, ProtocolMessage::Timeout);
    }

    #[test]
    fn flow_control_timeout_fails() {
        let mut h = TxProtocolHandler::new(3, 10);
        h.start();
        h.frame_sending();
        h.frames_sent(); // → Wait
        let t = h.expired();
        assert_eq!(h.state(), TxState::Fail);
        assert_eq!(t.message, ProtocolMessage::Timeout);
    }

    #[test]
    fn block_boundary_expects_flow_control() {
        // 4 frames, block_size = 2: after FF+2CFs → wait for FC
        let mut h = TxProtocolHandler::new(4, 10);
        h.start();
        h.frame_sending(); // FF, index → 1
        h.frames_sent();   // → Wait for initial FC

        // Receive CTS with block_size=2: block_end = 1 + 2 = 3
        h.handle_flow_control(FlowStatus::ContinueToSend, 2);
        assert_eq!(h.state(), TxState::Send);

        // Send CF1 (index 1→2)
        h.frame_sending();
        h.frames_sent(); // index=2 < block_end=3 → stay in Send

        // Send CF2 (index 2→3)
        h.frame_sending();
        let t = h.frames_sent(); // index=3 == block_end=3 → Wait
        assert!(t.state_changed);
        assert_eq!(h.state(), TxState::Wait);
    }

    #[test]
    fn frame_index_advances() {
        let mut h = TxProtocolHandler::new(5, 10);
        h.start();
        assert_eq!(h.frame_index(), 0);
        h.frame_sending();
        assert_eq!(h.frame_index(), 1);
        h.frames_sent(); // → Wait
        h.handle_flow_control(FlowStatus::ContinueToSend, 0);
        h.frame_sending();
        assert_eq!(h.frame_index(), 2);
    }

    #[test]
    fn separation_time_stored_on_cts() {
        let mut h = TxProtocolHandler::new(3, 10);
        h.start();
        h.frame_sending();
        h.frames_sent();
        let t = h.handle_flow_control(FlowStatus::ContinueToSend, 0);
        assert!(t.actions.store_separation_time);
    }

    #[test]
    fn cancel_on_fail() {
        let mut h = TxProtocolHandler::new(3, 10);
        h.start();
        h.frame_sending();
        h.frames_sent();
        let t = h.expired();
        assert!(t.actions.cancel_pending_send);
    }

    #[test]
    fn complete_multi_frame_sequence() {
        // 3-frame message: FF + CF1 + CF2, unlimited block
        let mut h = TxProtocolHandler::new(3, 10);
        h.start();

        // Send FF
        h.frame_sending();    // index → 1
        h.frames_sent();      // → Wait

        // Receive CTS, unlimited block
        h.handle_flow_control(FlowStatus::ContinueToSend, 0);
        assert_eq!(h.state(), TxState::Send);

        // Send CF1
        h.frame_sending();    // index → 2
        h.frames_sent();      // still in Send

        // Send CF2
        h.frame_sending();    // index → 3
        let t = h.frames_sent(); // index=3 == frame_count=3 → Success
        assert!(t.state_changed);
        assert_eq!(h.state(), TxState::Success);
        assert!(h.is_complete());
    }
}
