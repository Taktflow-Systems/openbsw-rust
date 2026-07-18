use bsw_can::transceiver::{CanTransceiver, ErrorCode};
use bsw_can::virtual_bus::VirtualCanBus;
use bsw_docan::addressing::{
    Addressing, ConnectionInfo, DataLinkAddressPair, TransportAddressPair,
};
use bsw_docan::codec::{encode_consecutive_frame, encode_first_frame};
use bsw_docan::constants::FlowStatus;
use bsw_docan::parameters::Parameters;
use bsw_docan::transport::{
    ConnectionConfig, ReceiveAction, RxSession, RxSessionState, SessionError, TimerKind, TxSession,
    TxSessionState,
};
use bsw_docan::virtual_transport::VirtualCanTransport;
use bsw_time::{Duration, Instant};

fn params() -> Parameters {
    Parameters {
        wait_allocate_timeout_us: 1_000,
        wait_rx_timeout_us: 3_000,
        wait_tx_callback_timeout_us: 2_000,
        wait_flow_control_timeout_us: 4_000,
        wait_consecutive_send_timeout_us: 10_000,
        max_allocate_retry_count: 2,
        max_flow_control_wait_count: 2,
        min_separation_time_us: 0,
        max_block_size: 0,
    }
}

fn config(addressing: Addressing, rx: u32, tx: u32) -> ConnectionConfig {
    ConnectionConfig {
        connection: ConnectionInfo::new(
            TransportAddressPair::new(0x00f1, 0x0006),
            DataLinkAddressPair::new(rx, tx),
        ),
        addressing,
        frame_size: 8,
        filler_byte: 0xcc,
        block_size: 2,
        parameters: params(),
    }
}

fn us(value: u64) -> Instant {
    Instant::from_nanos(value * 1_000)
}

#[test]
fn normal_extended_and_mixed_codec_vectors_cover_classic_layouts() {
    let vectors = [
        (Addressing::normal(), vec![0x03, 1, 2, 3]),
        (Addressing::extended(0xf1), vec![0xf1, 0x03, 1, 2, 3]),
        (Addressing::mixed(0x99), vec![0x99, 0x03, 1, 2, 3]),
    ];
    for (addressing, expected) in vectors {
        let mut tx: TxSession<32> = TxSession::new(config(addressing, 0x708, 0x700));
        tx.start(&[1, 2, 3]).unwrap();
        let frame = tx.poll(us(0)).unwrap().unwrap();
        assert_eq!(&frame.bytes()[..expected.len()], expected);
        tx.confirm(true, us(1)).unwrap();
        assert_eq!(tx.state(), TxSessionState::Complete);
    }
}

#[test]
fn tx_n_as_n_bs_n_cs_and_stmin_boundaries_use_injected_time() {
    let cfg = config(Addressing::normal(), 0x708, 0x700);
    let payload: Vec<u8> = (0..20).collect();

    let mut nas: TxSession<64> = TxSession::new(cfg);
    nas.start(&payload).unwrap();
    nas.poll(us(0)).unwrap().unwrap();
    assert_eq!(nas.poll(us(1_999)).unwrap(), None);
    assert_eq!(
        nas.poll(us(2_000)),
        Err(SessionError::Timeout(TimerKind::NAs))
    );

    let mut nbs: TxSession<64> = TxSession::new(cfg);
    nbs.start(&payload).unwrap();
    nbs.poll(us(0)).unwrap();
    nbs.confirm(true, us(10)).unwrap();
    assert_eq!(nbs.poll(us(4_009)).unwrap(), None);
    assert_eq!(
        nbs.poll(us(4_010)),
        Err(SessionError::Timeout(TimerKind::NBs))
    );

    let mut pacing: TxSession<64> = TxSession::new(cfg);
    pacing.start(&payload).unwrap();
    pacing.poll(us(0)).unwrap();
    pacing.confirm(true, us(10)).unwrap();
    pacing
        .flow_control(FlowStatus::ContinueToSend, 0, 5, us(20))
        .unwrap();
    assert_eq!(pacing.poll(us(5_019)).unwrap(), None);
    assert!(pacing.poll(us(5_020)).unwrap().is_some());

    let mut ncs_cfg = cfg;
    ncs_cfg.parameters.wait_consecutive_send_timeout_us = 4_000;
    let mut ncs: TxSession<64> = TxSession::new(ncs_cfg);
    ncs.start(&payload).unwrap();
    ncs.poll(us(0)).unwrap();
    ncs.confirm(true, us(10)).unwrap();
    ncs.flow_control(FlowStatus::ContinueToSend, 0, 5, us(20))
        .unwrap();
    assert_eq!(
        ncs.poll(us(4_020)),
        Err(SessionError::Timeout(TimerKind::NCs))
    );
}

#[test]
fn tx_blocks_wait_overflow_cancel_and_recovery_are_bounded() {
    let cfg = config(Addressing::normal(), 0x708, 0x700);
    let payload: Vec<u8> = (0..40).collect();
    let mut tx: TxSession<64> = TxSession::new(cfg);
    tx.start(&payload).unwrap();
    tx.poll(us(0)).unwrap();
    tx.confirm(true, us(1)).unwrap();
    tx.flow_control(FlowStatus::Wait, 0, 0, us(2)).unwrap();
    assert_eq!(
        tx.flow_control(FlowStatus::Wait, 0, 0, us(3)),
        Err(SessionError::WaitCountExceeded)
    );
    assert_eq!(
        tx.state(),
        TxSessionState::Failed(SessionError::WaitCountExceeded)
    );

    tx.start(&payload).unwrap();
    tx.poll(us(4)).unwrap();
    tx.confirm(true, us(5)).unwrap();
    assert_eq!(
        tx.flow_control(FlowStatus::Overflow, 0, 0, us(6)),
        Err(SessionError::Overflow)
    );
    tx.start(&[0x3e, 0]).unwrap();
    tx.cancel();
    assert_eq!(tx.state(), TxSessionState::Failed(SessionError::Cancelled));
    tx.start(&[0x10, 1]).unwrap();
    assert!(tx.poll(us(7)).unwrap().is_some());
}

fn first_frame(cfg: ConnectionConfig, length: u32, data: &[u8]) -> [u8; 8] {
    let mut frame = [cfg.filler_byte; 8];
    cfg.addressing.write_prefix(&mut frame);
    encode_first_frame(
        &mut frame,
        length,
        data,
        &cfg.addressing.codec(cfg.filler_byte),
    )
    .unwrap();
    frame
}

fn consecutive(cfg: ConnectionConfig, sequence: u8, data: &[u8]) -> [u8; 8] {
    let mut frame = [cfg.filler_byte; 8];
    cfg.addressing.write_prefix(&mut frame);
    encode_consecutive_frame(
        &mut frame,
        sequence,
        data,
        &cfg.addressing.codec(cfg.filler_byte),
    )
    .unwrap();
    frame
}

#[test]
fn rx_n_br_retries_overflow_n_cr_and_sequence_recovery() {
    let cfg = config(Addressing::normal(), 0x700, 0x708);
    let mut rx: RxSession<32> = RxSession::new(cfg);
    let first = first_frame(cfg, 20, &[0, 1, 2, 3, 4, 5]);
    assert!(matches!(
        rx.receive(&first, us(0), false).unwrap(),
        ReceiveAction::FlowControl(_)
    ));
    assert_eq!(rx.state(), RxSessionState::WaitAllocation);
    assert_eq!(rx.poll(us(999), false).unwrap(), ReceiveAction::None);
    assert!(matches!(
        rx.poll(us(1_000), false).unwrap(),
        ReceiveAction::FlowControl(_)
    ));
    assert!(matches!(
        rx.poll(us(2_000), false).unwrap(),
        ReceiveAction::FlowControl(_)
    ));
    assert_eq!(rx.state(), RxSessionState::Failed(SessionError::Overflow));

    assert!(matches!(
        rx.receive(&first, us(3_000), true).unwrap(),
        ReceiveAction::FlowControl(_)
    ));
    let wrong = consecutive(cfg, 2, &[6, 7, 8, 9, 10, 11, 12]);
    assert_eq!(
        rx.receive(&wrong, us(3_100), true),
        Err(SessionError::WrongSequence)
    );

    assert!(matches!(
        rx.receive(&first, us(4_000), true).unwrap(),
        ReceiveAction::FlowControl(_)
    ));
    assert_eq!(rx.poll(us(6_999), true).unwrap(), ReceiveAction::None);
    assert_eq!(
        rx.poll(us(7_000), true),
        Err(SessionError::Timeout(TimerKind::NCr))
    );
}

#[test]
fn rx_sequence_wrap_reassembles_and_returns_pool_owned_message() {
    let mut cfg = config(Addressing::normal(), 0x700, 0x708);
    cfg.block_size = 0;
    let original: Vec<u8> = (0..125).collect();
    let mut rx: RxSession<160> = RxSession::new(cfg);
    assert!(matches!(
        rx.receive(&first_frame(cfg, 125, &original[..6]), us(0), true)
            .unwrap(),
        ReceiveAction::FlowControl(_)
    ));
    let mut offset = 6;
    let mut sequence = 1u8;
    while offset < original.len() {
        let end = (offset + 7).min(original.len());
        let action = rx
            .receive(
                &consecutive(cfg, sequence, &original[offset..end]),
                us(offset as u64),
                true,
            )
            .unwrap();
        offset = end;
        sequence = sequence.wrapping_add(1) & 0x0f;
        if offset == original.len() {
            assert_eq!(action, ReceiveAction::Complete);
        }
    }
    let message = rx.take_message().unwrap();
    assert_eq!(message.payload(), original);
    assert_eq!(message.source_address(), 0x00f1);
    assert_eq!(message.target_address(), 0x0006);
}

fn open_node(bus: &VirtualCanBus) -> bsw_can::virtual_bus::VirtualNode {
    let mut node = bus.add_node();
    assert_eq!(node.init(), ErrorCode::Ok);
    assert_eq!(node.open(), ErrorCode::Ok);
    node
}

#[test]
fn platform_independent_transport_runs_multiframe_over_virtual_bus() {
    let bus = VirtualCanBus::new();
    let a = open_node(&bus);
    let b = open_node(&bus);
    let mut sender: VirtualCanTransport<256> =
        VirtualCanTransport::new(a, config(Addressing::normal(), 0x708, 0x700));
    let mut receiver: VirtualCanTransport<256> =
        VirtualCanTransport::new(b, config(Addressing::normal(), 0x700, 0x708));
    let payload: Vec<u8> = (0..125).collect();
    sender.send(&payload).unwrap();

    let mut now = Instant::from_nanos(0);
    for _ in 0..100 {
        sender.cycle(now).unwrap();
        bus.step();
        receiver.cycle(now).unwrap();
        bus.step();
        now = now.wrapping_add(Duration::from_micros(100).unwrap());
        if let Some(message) = receiver.take_received() {
            assert_eq!(message.payload(), payload);
            assert_eq!(sender.tx().state(), TxSessionState::Complete);
            return;
        }
    }
    panic!("virtual CAN transfer did not complete");
}
