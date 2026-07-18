//! Deterministic in-process virtual CAN bus (package D16).
//!
//! Protocol tests attach several [`VirtualNode`]s to one [`VirtualCanBus`]
//! and drive delivery explicitly with [`VirtualCanBus::step`] — no shell
//! subprocesses, no timing races. Delivery is arbitration-neutral: frames
//! are delivered to every other open node in submission order, without ID
//! priority.
//!
//! Fault injection covers the failure modes protocol suites need:
//! per-node delivery delay (in steps), scripted frame drops, injected
//! error events, and forced bus-off with explicit recovery.

use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;

use crate::error_state::ErrorStateTracker;
use crate::frame::CanFrame;
use crate::transceiver::{CanTransceiver, ErrorCode, State, TransceiverState};

#[derive(Debug)]
struct NodeCore {
    state: State,
    error_state: ErrorStateTracker,
    rx: VecDeque<CanFrame>,
    /// Frames waiting `delay` further steps before delivery to this node.
    in_flight: VecDeque<(u32, CanFrame)>,
    delay_steps: u32,
    drop_next: u32,
    dropped: u32,
    tx_count: u32,
    rx_count: u32,
    forced_bus_off: bool,
}

impl NodeCore {
    fn new() -> Self {
        Self {
            state: State::Closed,
            error_state: ErrorStateTracker::new(),
            rx: VecDeque::new(),
            in_flight: VecDeque::new(),
            delay_steps: 0,
            drop_next: 0,
            dropped: 0,
            tx_count: 0,
            rx_count: 0,
            forced_bus_off: false,
        }
    }
}

#[derive(Debug)]
struct BusCore {
    nodes: Vec<NodeCore>,
    /// Frames transmitted since the last step, in submission order.
    pending: Vec<(usize, CanFrame)>,
}

/// Shared in-process CAN bus.
#[derive(Clone)]
pub struct VirtualCanBus {
    core: Rc<RefCell<BusCore>>,
}

impl VirtualCanBus {
    /// Create a bus with no nodes.
    #[must_use]
    pub fn new() -> Self {
        Self {
            core: Rc::new(RefCell::new(BusCore {
                nodes: Vec::new(),
                pending: Vec::new(),
            })),
        }
    }

    /// Attach a new node in `Closed` state.
    #[must_use]
    pub fn add_node(&self) -> VirtualNode {
        let mut core = self.core.borrow_mut();
        core.nodes.push(NodeCore::new());
        VirtualNode {
            core: Rc::clone(&self.core),
            index: core.nodes.len() - 1,
        }
    }

    /// Deliver pending traffic: advance delayed frames one step and move
    /// due frames into receiver queues. Returns delivered frame count.
    pub fn step(&self) -> usize {
        let mut core = self.core.borrow_mut();
        let pending = std::mem::take(&mut core.pending);
        // Stage newly transmitted frames toward every other open node.
        for (sender, frame) in pending {
            let node_count = core.nodes.len();
            for receiver in 0..node_count {
                if receiver == sender {
                    continue;
                }
                let node = &mut core.nodes[receiver];
                if node.state != State::Open && node.state != State::Muted {
                    continue;
                }
                if node.drop_next > 0 {
                    node.drop_next -= 1;
                    node.dropped += 1;
                    continue;
                }
                node.in_flight.push_back((node.delay_steps, frame.clone()));
            }
        }
        // Advance in-flight frames and deliver the due ones.
        let mut delivered = 0;
        for node in &mut core.nodes {
            let mut still_flying = VecDeque::new();
            while let Some((steps, frame)) = node.in_flight.pop_front() {
                if steps == 0 {
                    node.rx.push_back(frame);
                    node.rx_count += 1;
                    node.error_state.record_rx_success();
                    delivered += 1;
                } else {
                    still_flying.push_back((steps - 1, frame));
                }
            }
            node.in_flight = still_flying;
        }
        delivered
    }

    /// Run `step` until no traffic is in flight.
    pub fn settle(&self) -> usize {
        let mut delivered = 0;
        loop {
            let now = self.step();
            delivered += now;
            let core = self.core.borrow();
            if core.pending.is_empty() && core.nodes.iter().all(|node| node.in_flight.is_empty()) {
                return delivered;
            }
            drop(core);
        }
    }
}

impl Default for VirtualCanBus {
    fn default() -> Self {
        Self::new()
    }
}

/// One transceiver attached to a [`VirtualCanBus`].
pub struct VirtualNode {
    core: Rc<RefCell<BusCore>>,
    index: usize,
}

impl VirtualNode {
    /// Pop the next received frame.
    pub fn receive(&mut self) -> Option<CanFrame> {
        self.core.borrow_mut().nodes[self.index].rx.pop_front()
    }

    /// Frames received so far.
    pub fn rx_count(&self) -> u32 {
        self.core.borrow().nodes[self.index].rx_count
    }

    /// Frames transmitted so far.
    pub fn tx_count(&self) -> u32 {
        self.core.borrow().nodes[self.index].tx_count
    }

    /// Frames dropped by injection at this node.
    pub fn dropped(&self) -> u32 {
        self.core.borrow().nodes[self.index].dropped
    }

    /// Delay every future delivery to this node by `steps` bus steps.
    pub fn set_delivery_delay(&mut self, steps: u32) {
        self.core.borrow_mut().nodes[self.index].delay_steps = steps;
    }

    /// Drop the next `count` frames that would be delivered to this node.
    pub fn inject_drop(&mut self, count: u32) {
        self.core.borrow_mut().nodes[self.index].drop_next += count;
    }

    /// Inject a burst of transmit errors, driving this node's error state
    /// toward passive and bus off exactly like real fault confinement.
    pub fn inject_tx_errors(&mut self, count: u32) -> TransceiverState {
        let mut core = self.core.borrow_mut();
        let node = &mut core.nodes[self.index];
        for _ in 0..count {
            node.error_state.record_tx_error();
        }
        node.error_state.state()
    }

    /// Force the node bus off immediately.
    pub fn force_bus_off(&mut self) {
        let mut core = self.core.borrow_mut();
        let node = &mut core.nodes[self.index];
        node.forced_bus_off = true;
        while node.error_state.state() != TransceiverState::BusOff {
            node.error_state.record_tx_error();
        }
    }

    /// Recover a bus-off node: counters reset, node stays in its logical
    /// state and may transmit again.
    pub fn recover_bus_off(&mut self) {
        let mut core = self.core.borrow_mut();
        let node = &mut core.nodes[self.index];
        node.forced_bus_off = false;
        node.error_state = ErrorStateTracker::new();
    }
}

impl CanTransceiver for VirtualNode {
    fn init(&mut self) -> ErrorCode {
        let mut core = self.core.borrow_mut();
        let node = &mut core.nodes[self.index];
        if node.state != State::Closed {
            return ErrorCode::IllegalState;
        }
        node.state = State::Initialized;
        ErrorCode::Ok
    }

    fn shutdown(&mut self) {
        let mut core = self.core.borrow_mut();
        let node = &mut core.nodes[self.index];
        node.state = State::Closed;
        node.rx.clear();
        node.in_flight.clear();
    }

    fn open(&mut self) -> ErrorCode {
        let mut core = self.core.borrow_mut();
        let node = &mut core.nodes[self.index];
        if node.state != State::Initialized && node.state != State::Muted {
            return ErrorCode::IllegalState;
        }
        node.state = State::Open;
        ErrorCode::Ok
    }

    fn open_with_frame(&mut self, frame: &CanFrame) -> ErrorCode {
        let code = self.open();
        if code != ErrorCode::Ok {
            return code;
        }
        self.write(frame)
    }

    fn close(&mut self) -> ErrorCode {
        let mut core = self.core.borrow_mut();
        let node = &mut core.nodes[self.index];
        if node.state == State::Closed {
            return ErrorCode::IllegalState;
        }
        node.state = State::Initialized;
        ErrorCode::Ok
    }

    fn mute(&mut self) -> ErrorCode {
        let mut core = self.core.borrow_mut();
        let node = &mut core.nodes[self.index];
        if node.state != State::Open {
            return ErrorCode::IllegalState;
        }
        node.state = State::Muted;
        ErrorCode::Ok
    }

    fn unmute(&mut self) -> ErrorCode {
        let mut core = self.core.borrow_mut();
        let node = &mut core.nodes[self.index];
        if node.state != State::Muted {
            return ErrorCode::IllegalState;
        }
        node.state = State::Open;
        ErrorCode::Ok
    }

    fn state(&self) -> State {
        self.core.borrow().nodes[self.index].state
    }

    fn baudrate(&self) -> u32 {
        crate::transceiver::BAUDRATE_HIGHSPEED
    }

    fn hw_queue_timeout(&self) -> u16 {
        0
    }

    fn bus_id(&self) -> u8 {
        #[allow(clippy::cast_possible_truncation)]
        {
            self.index as u8
        }
    }

    fn write(&mut self, frame: &CanFrame) -> ErrorCode {
        let mut core = self.core.borrow_mut();
        let index = self.index;
        {
            let node = &mut core.nodes[index];
            if node.state == State::Muted {
                return ErrorCode::TxOffline;
            }
            if node.state != State::Open {
                return ErrorCode::TxOffline;
            }
            if node.error_state.state() == TransceiverState::BusOff || node.forced_bus_off {
                return ErrorCode::TxFail;
            }
            node.tx_count += 1;
            node.error_state.record_tx_success();
        }
        core.pending.push((index, frame.clone()));
        ErrorCode::Ok
    }

    fn transceiver_state(&self) -> TransceiverState {
        self.core.borrow().nodes[self.index].error_state.state()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::can_id::CanId;

    fn open_node(bus: &VirtualCanBus) -> VirtualNode {
        let mut node = bus.add_node();
        assert_eq!(node.init(), ErrorCode::Ok);
        assert_eq!(node.open(), ErrorCode::Ok);
        node
    }

    fn frame(raw_id: u16, data: &[u8]) -> CanFrame {
        CanFrame::with_data(CanId::base(raw_id), data)
    }

    #[test]
    fn frames_reach_every_other_open_node_in_order() {
        let bus = VirtualCanBus::new();
        let mut alpha = open_node(&bus);
        let mut beta = open_node(&bus);
        let gamma = open_node(&bus);
        assert_eq!(alpha.write(&frame(0x700, &[1])), ErrorCode::Ok);
        assert_eq!(alpha.write(&frame(0x100, &[2])), ErrorCode::Ok);
        assert_eq!(bus.step(), 4);
        // Submission order is preserved: no ID-based arbitration.
        assert_eq!(beta.receive().unwrap().id(), CanId::base(0x700));
        assert_eq!(beta.receive().unwrap().id(), CanId::base(0x100));
        assert_eq!(gamma.rx_count(), 2);
        assert!(alpha.receive().is_none(), "no self-reception");
    }

    #[test]
    fn closed_and_muted_nodes_do_not_receive_but_muted_cannot_send() {
        let bus = VirtualCanBus::new();
        let mut sender = open_node(&bus);
        let mut muted = open_node(&bus);
        let mut closed = bus.add_node();
        assert_eq!(muted.mute(), ErrorCode::Ok);
        assert_eq!(muted.write(&frame(1, &[])), ErrorCode::TxOffline);
        sender.write(&frame(2, &[]));
        bus.step();
        // Muted nodes still listen (upstream mute is TX-only).
        assert_eq!(muted.rx_count(), 1);
        assert_eq!(closed.receive(), None);
        assert_eq!(muted.unmute(), ErrorCode::Ok);
        assert_eq!(muted.write(&frame(3, &[])), ErrorCode::Ok);
    }

    #[test]
    fn delivery_delay_holds_frames_for_n_steps() {
        let bus = VirtualCanBus::new();
        let mut sender = open_node(&bus);
        let mut slow = open_node(&bus);
        slow.set_delivery_delay(2);
        sender.write(&frame(5, &[9]));
        assert_eq!(bus.step(), 0);
        assert_eq!(bus.step(), 0);
        assert_eq!(bus.step(), 1);
        assert_eq!(slow.receive().unwrap().payload(), &[9]);
    }

    #[test]
    fn drop_injection_loses_exactly_the_scripted_frames() {
        let bus = VirtualCanBus::new();
        let mut sender = open_node(&bus);
        let mut lossy = open_node(&bus);
        lossy.inject_drop(2);
        for value in 0..4u8 {
            sender.write(&frame(u16::from(value), &[value]));
        }
        bus.settle();
        assert_eq!(lossy.dropped(), 2);
        assert_eq!(lossy.receive().unwrap().payload(), &[2]);
        assert_eq!(lossy.receive().unwrap().payload(), &[3]);
        assert_eq!(lossy.receive(), None);
    }

    #[test]
    fn error_injection_walks_to_bus_off_and_blocks_tx() {
        let bus = VirtualCanBus::new();
        let mut node = open_node(&bus);
        let _receiver = open_node(&bus);
        assert_eq!(node.inject_tx_errors(16), TransceiverState::Passive);
        assert_eq!(node.inject_tx_errors(16), TransceiverState::BusOff);
        assert_eq!(node.write(&frame(1, &[])), ErrorCode::TxFail);
        node.recover_bus_off();
        assert_eq!(node.transceiver_state(), TransceiverState::Active);
        assert_eq!(node.write(&frame(1, &[])), ErrorCode::Ok);
    }

    #[test]
    fn forced_bus_off_and_recovery_round_trip() {
        let bus = VirtualCanBus::new();
        let mut node = open_node(&bus);
        node.force_bus_off();
        assert_eq!(node.transceiver_state(), TransceiverState::BusOff);
        assert_eq!(node.write(&frame(1, &[])), ErrorCode::TxFail);
        node.recover_bus_off();
        assert_eq!(node.write(&frame(1, &[])), ErrorCode::Ok);
    }

    #[test]
    fn lifecycle_state_machine_matches_transceiver_contract() {
        let bus = VirtualCanBus::new();
        let mut node = bus.add_node();
        assert_eq!(node.state(), State::Closed);
        assert_eq!(node.open(), ErrorCode::IllegalState);
        assert_eq!(node.init(), ErrorCode::Ok);
        assert_eq!(node.init(), ErrorCode::IllegalState);
        assert_eq!(node.open(), ErrorCode::Ok);
        assert_eq!(node.close(), ErrorCode::Ok);
        assert_eq!(node.state(), State::Initialized);
        node.shutdown();
        assert_eq!(node.state(), State::Closed);
    }

    #[test]
    fn open_with_frame_transmits_immediately() {
        let bus = VirtualCanBus::new();
        let mut sender = bus.add_node();
        let mut receiver = open_node(&bus);
        sender.init();
        assert_eq!(sender.open_with_frame(&frame(0x55, &[7])), ErrorCode::Ok);
        bus.settle();
        assert_eq!(receiver.receive().unwrap().payload(), &[7]);
    }
}
