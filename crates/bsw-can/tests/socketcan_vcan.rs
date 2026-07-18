//! SocketCAN integration against a virtual CAN interface (package D15).
//!
//! Runs only on Linux with the `socketcan` feature. CI provisions `vcan0`
//! (with an FD-capable MTU); on machines without it every test skips with
//! a printed note instead of failing, so local runs stay green.

#![cfg(all(feature = "socketcan", target_os = "linux"))]

use bsw_can::socketcan::{KernelFilter, RxEvent, SocketCan};
use bsw_can::{CanFrame, CanId};
use std::sync::{Mutex, MutexGuard};

const VCAN: &str = "vcan0";
static VCAN_LOCK: Mutex<()> = Mutex::new(());

fn isolated_vcan() -> MutexGuard<'static, ()> {
    VCAN_LOCK.lock().expect("vcan test lock poisoned")
}

fn open_pair() -> Option<(SocketCan, SocketCan)> {
    if let (Ok(tx), Ok(rx)) = (SocketCan::open(VCAN), SocketCan::open(VCAN)) {
        Some((tx, rx))
    } else {
        eprintln!("note: {VCAN} not available; skipping SocketCAN integration test");
        None
    }
}

fn recv_with_retry(socket: &mut SocketCan) -> Option<RxEvent> {
    // vcan delivery is immediate, but give the scheduler a little room.
    for _ in 0..100 {
        match socket.receive() {
            Ok(Some(event)) => return Some(event),
            Ok(None) => std::thread::sleep(std::time::Duration::from_millis(1)),
            Err(error) => panic!("receive failed: {error}"),
        }
    }
    None
}

#[test]
fn classic_frame_roundtrip_with_timestamp() {
    let _guard = isolated_vcan();
    let Some((tx, mut rx)) = open_pair() else {
        return;
    };
    let frame = CanFrame::with_data(CanId::base(0x321), &[1, 2, 3, 4]);
    tx.send(&frame).unwrap();
    let event = recv_with_retry(&mut rx).expect("frame not delivered");
    let RxEvent::Frame(received) = event else {
        panic!("expected data frame, got {event:?}");
    };
    assert_eq!(received.id(), CanId::base(0x321));
    assert_eq!(received.payload(), &[1, 2, 3, 4]);
    assert_ne!(received.timestamp(), 0, "kernel timestamp missing");
}

#[test]
fn extended_ids_survive_the_wire() {
    let _guard = isolated_vcan();
    let Some((tx, mut rx)) = open_pair() else {
        return;
    };
    let frame = CanFrame::with_data(CanId::extended(0x0ABC_DEF0), &[9]);
    tx.send(&frame).unwrap();
    let RxEvent::Frame(received) = recv_with_retry(&mut rx).expect("frame not delivered") else {
        panic!("expected data frame");
    };
    assert_eq!(received.id(), CanId::extended(0x0ABC_DEF0));
}

#[test]
fn kernel_filters_reject_and_accept() {
    let _guard = isolated_vcan();
    let Some((tx, mut rx)) = open_pair() else {
        return;
    };
    rx.set_filters(&[KernelFilter::exact(CanId::base(0x100))])
        .unwrap();
    tx.send(&CanFrame::with_data(CanId::base(0x200), &[1]))
        .unwrap();
    tx.send(&CanFrame::with_data(CanId::base(0x100), &[2]))
        .unwrap();
    let RxEvent::Frame(received) = recv_with_retry(&mut rx).expect("accepted frame missing") else {
        panic!("expected data frame");
    };
    assert_eq!(
        received.id(),
        CanId::base(0x100),
        "filter let 0x200 through"
    );
    // Nothing else must arrive.
    assert!(rx.receive().unwrap().is_none());
    // Re-opening the filter restores delivery.
    rx.open_filter().unwrap();
    tx.send(&CanFrame::with_data(CanId::base(0x300), &[3]))
        .unwrap();
    assert!(recv_with_retry(&mut rx).is_some());
}

#[test]
fn loopback_control_isolates_local_sockets() {
    let _guard = isolated_vcan();
    let Some((tx, mut rx)) = open_pair() else {
        return;
    };
    tx.set_loopback(false).unwrap();
    tx.send(&CanFrame::with_data(CanId::base(0x42), &[7]))
        .unwrap();
    // With loopback off, the sibling socket must not see the frame.
    std::thread::sleep(std::time::Duration::from_millis(5));
    assert!(rx.receive().unwrap().is_none());
}

#[test]
fn receive_own_messages_control() {
    let _guard = isolated_vcan();
    let Some((mut tx, _rx)) = open_pair() else {
        return;
    };
    tx.set_receive_own(true).unwrap();
    tx.send(&CanFrame::with_data(CanId::base(0x50), &[5]))
        .unwrap();
    let event = recv_with_retry(&mut tx).expect("own frame not echoed");
    let RxEvent::Frame(received) = event else {
        panic!("expected data frame");
    };
    assert_eq!(received.payload(), &[5]);
}

#[cfg(feature = "can-fd")]
#[test]
fn fd_frames_roundtrip_when_the_interface_allows_them() {
    let _guard = isolated_vcan();
    let (Ok(tx), Ok(mut rx)) = (SocketCan::open_fd(VCAN), SocketCan::open_fd(VCAN)) else {
        eprintln!("note: {VCAN} not available; skipping FD test");
        return;
    };
    let payload: Vec<u8> = (0..48u8).collect();
    let frame = CanFrame::with_data(CanId::base(0x600), &payload);
    if let Err(error) = tx.send(&frame) {
        // MTU below CANFD_MTU: environment cannot carry FD frames.
        eprintln!("note: FD send rejected ({error}); interface lacks FD MTU");
        return;
    }
    let RxEvent::Frame(received) = recv_with_retry(&mut rx).expect("FD frame not delivered") else {
        panic!("expected data frame");
    };
    assert_eq!(received.payload(), payload.as_slice());
}
