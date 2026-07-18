// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

use bsw_com::{
    ByteOrder, ComE2eError, ComManager, PduDescriptor, SignalDescriptor, SignalType, SignalValue,
    TxMode,
};
use bsw_time::{Duration, Instant};
use bsw_util::e2e::{E2eChecker, E2eProfile, E2eProtector, E2eResult};

fn signal() -> SignalDescriptor {
    SignalDescriptor::new(7, 16, 8, SignalType::Uint8, ByteOrder::LittleEndian, 0)
}

#[test]
fn project_e2e_round_trip_integrates_with_com_state() {
    let mut tx: ComManager<1, 1> = ComManager::new();
    let tx_pdu = tx
        .add_pdu(PduDescriptor::tx(0x321, 8, TxMode::EventOnly), &[signal()])
        .unwrap();
    tx.write_signal(7, SignalValue::U8(0x5A));
    let mut protector = E2eProtector::new(E2eProfile::default());
    tx.protect_project_e2e(tx_pdu, &mut protector).unwrap();
    tx.start(Instant::from_nanos(0));
    assert!(tx.trigger(tx_pdu));
    let mut wire = [0u8; 8];
    wire.copy_from_slice(tx.tick(Instant::from_nanos(0)).next().unwrap().1);

    let mut rx: ComManager<1, 1> = ComManager::new();
    let rx_pdu = rx
        .add_pdu(
            PduDescriptor::rx(0x321, 8)
                .with_rx_timeout(Duration::from_nanos(10), bsw_com::RxTimeoutAction::KeepLast),
            &[signal()],
        )
        .unwrap();
    rx.start(Instant::from_nanos(0));
    let mut checker = E2eChecker::new(E2eProfile::default());
    assert_eq!(
        rx.receive_project_e2e(0x321, &wire, Instant::from_nanos(5), &mut checker),
        Ok(E2eResult::Initial)
    );
    assert_eq!(rx.read_signal(7), Some(SignalValue::U8(0x5A)));
    assert_eq!(rx.take_pdu_received(rx_pdu), Some(true));
    assert_eq!(rx.is_signal_valid(7), Some(true));

    wire[4] ^= 1;
    assert_eq!(
        rx.receive_project_e2e(0x321, &wire, Instant::from_nanos(8), &mut checker),
        Ok(E2eResult::WrongCrc)
    );
    assert_eq!(rx.is_signal_valid(7), Some(false));
    assert_eq!(rx.take_pdu_received(rx_pdu), Some(false));
}

#[test]
fn project_e2e_integration_rejects_direction_and_length_errors() {
    let mut com: ComManager<1, 1> = ComManager::new();
    let pdu = com
        .add_pdu(PduDescriptor::tx(0x123, 8, TxMode::EventOnly), &[signal()])
        .unwrap();
    let mut checker = E2eChecker::new(E2eProfile::default());
    assert_eq!(
        com.receive_project_e2e(0x123, &[0; 8], Instant::from_nanos(0), &mut checker),
        Err(ComE2eError::WrongDirection)
    );

    let mut protector = E2eProtector::new(E2eProfile::default());
    assert_eq!(com.protect_project_e2e(pdu, &mut protector), Ok(()));
    assert_eq!(
        com.protect_project_e2e(1, &mut protector),
        Err(ComE2eError::UnknownPdu)
    );
}
