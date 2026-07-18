use std::sync::{Arc, Mutex};
use std::thread;

use bsw_can::{CanFrame, CanId};
use bsw_doip::{DiagnosticPayload, Packet, Payload, ProtocolVersion};
use bsw_storage::block::{BlockId, BlockStore};
use bsw_storage::fault::{CutPlan, CutPointBackend};
use bsw_storage::journal::JournalStore;
use bsw_storage::mem::MemBackend;
use bsw_time::{Duration, Instant};
use bsw_uds::DiagSession;
use openbsw_reference_app::can_demo::CanDemo;
use openbsw_reference_app::diagnostics::{
    doip_request, virtual_can_request, ClientLimiter, DiagnosticCore, DiagnosticTransport,
};
use openbsw_reference_app::io::{IoError, SimulatedIo, ADC_MAX, PWM_MAX_PERMILLE};
use openbsw_reference_app::scenario::{oracle_json, run_getting_started_workflows};
use openbsw_reference_app::storage::PersistentServices;
use openbsw_reference_app::{AppConfig, ReferenceApp};

#[test]
fn f02_configuration_is_separate_and_drives_runtime_output() {
    let mut config = AppConfig::default();
    config.diagnostics.can_request_id = 0x700;
    config.diagnostics.can_response_id = 0x708;
    let mut app = ReferenceApp::new(config).unwrap();
    assert_eq!(
        app.command("can info", Instant::from_nanos(0)),
        "request=700 response=708"
    );
}

#[test]
fn f03_startup_run_shutdown_logs_are_stable_and_restartable() {
    let mut app = ReferenceApp::new(AppConfig::default()).unwrap();
    app.start(Instant::from_nanos(1_000_000)).unwrap();
    assert_eq!(app.level(), 9);
    assert_eq!(
        app.logs(),
        [
            "1: RefApp: RUNTIME: INFO: hello",
            "1: RefApp: RUNTIME: INFO: run level 9 reached"
        ]
    );
    app.shutdown(Instant::from_nanos(2_000_000)).unwrap();
    assert_eq!(app.level(), 0);
    app.start(Instant::from_nanos(3_000_000)).unwrap();
    assert_eq!(app.level(), 9);
}

#[test]
fn f03_console_tree_and_log_level_are_scriptable() {
    let mut app = ReferenceApp::new(AppConfig::default()).unwrap();
    let now = Instant::from_nanos(0);
    assert!(app.command("help", now).contains("lc level"));
    assert_eq!(app.command("logger level warn", now), "logger WARN");
    assert_eq!(app.command("lc level 5", now), "level 5");
    assert_eq!(app.command("stats all", now), "commands=4 diagnostics=0");
}

#[test]
fn f04_simulated_io_is_deterministic_and_bounded() {
    let mut io = SimulatedIo::default();
    assert_eq!(io.drive_adc(0).unwrap().pwm_permille, 0);
    assert_eq!(
        io.drive_adc(ADC_MAX).unwrap().pwm_permille,
        PWM_MAX_PERMILLE
    );
    assert_eq!(io.drive_adc(ADC_MAX + 1), Err(IoError::AdcOutOfRange));
    io.set_output(true);
    assert!(io.snapshot().output);
    assert_eq!(
        io.set_pwm(PWM_MAX_PERMILLE + 1),
        Err(IoError::PwmOutOfRange)
    );
}

#[test]
fn f05_cf01_multiframe_diagnostic_runs_over_virtual_can() {
    let config = AppConfig::default().diagnostics;
    let mut core = DiagnosticCore::new();
    let response = virtual_can_request(
        &mut core,
        config,
        &[0x22, 0xcf, 0x01],
        Instant::from_nanos(0),
    )
    .unwrap();
    assert_eq!(&response.bytes()[..3], &[0x62, 0xcf, 0x01]);
    assert_eq!(response.bytes().len(), 27);
    assert_eq!(core.dispatch_count(), 1);
}

#[test]
fn f05_configured_write_did_and_routine_work_over_virtual_can() {
    let config = AppConfig::default().diagnostics;
    let mut core = DiagnosticCore::new();
    assert_eq!(
        virtual_can_request(&mut core, config, &[0x10, 0x03], Instant::from_nanos(0))
            .unwrap()
            .bytes(),
        &[0x50, 0x03, 0x00, 0x32, 0x01, 0xf4]
    );
    assert_eq!(
        virtual_can_request(
            &mut core,
            config,
            &[0x2e, 0xcf, 0x03, 1, 2, 3, 4, 5, 6, 7, 8],
            Instant::from_nanos(1_000_000),
        )
        .unwrap()
        .bytes(),
        &[0x6e, 0xcf, 0x03]
    );
    assert_eq!(
        virtual_can_request(
            &mut core,
            config,
            &[0x31, 0x01, 0x12, 0x34],
            Instant::from_nanos(2_000_000),
        )
        .unwrap()
        .bytes(),
        &[0x71, 0x01, 0x12, 0x34, 0, 0, 0, 1]
    );
}

#[test]
fn f06_docan_and_doip_share_one_session_and_dispatch_counter() {
    let config = AppConfig::default().diagnostics;
    let mut core = DiagnosticCore::new();
    virtual_can_request(&mut core, config, &[0x10, 0x03], Instant::from_nanos(0)).unwrap();
    assert_eq!(core.session(), DiagSession::Extended);

    let request = Packet {
        version: ProtocolVersion::Iso2012,
        payload: Payload::DiagnosticMessage(DiagnosticPayload {
            source_address: config.tester_address,
            target_address: config.entity_address,
            data: &[0x22, 0xcf, 0x02],
        }),
    };
    let mut input = [0u8; 64];
    let input_len = request.encode(&mut input).unwrap();
    let mut output = [0u8; 64];
    let output_len = doip_request(
        &mut core,
        &input[..input_len],
        Instant::from_nanos(1),
        &mut output,
    )
    .unwrap();
    let response = Packet::parse(&output[..output_len]).unwrap();
    let Payload::DiagnosticMessage(response) = response.payload else {
        panic!("expected diagnostic response");
    };
    assert_eq!(response.data, &[0x62, 0xcf, 0x02, 0x03]);
    assert_eq!(core.dispatch_count(), 2);
}

#[test]
fn f06_udp_and_tcp_echo_work_on_real_loopback_sockets() {
    assert_eq!(
        openbsw_reference_app::network::udp_echo_once(b"udp").unwrap(),
        b"udp"
    );
    assert_eq!(
        openbsw_reference_app::network::tcp_echo_once(b"tcp").unwrap(),
        b"tcp"
    );
}

#[test]
fn f07_counter_and_blob_survive_journal_remount() {
    let mut services = PersistentServices::new().unwrap();
    services.write_counter(0x0a01, 42).unwrap();
    let blob: Vec<u8> = (0..64).collect();
    services.write_blob(&blob).unwrap();
    let services = services.restart().unwrap();
    assert_eq!(services.read_counter(0x0a01).unwrap(), Some(42));
    let mut output = [0u8; 64];
    assert_eq!(services.read_blob(&mut output).unwrap(), 64);
    assert_eq!(output, blob.as_slice());
}

#[test]
fn f07_torn_write_recovers_old_or_new_valid_value() {
    type Base = MemBackend<4096, 256, 4>;
    type Faulty = CutPointBackend<Base>;
    let backend = Faulty::new(Base::new());
    let mut store: JournalStore<Faulty, 8> = JournalStore::mount(backend).unwrap();
    store.write(BlockId(1), 1, b"old").unwrap();
    store.backend_mut().arm(CutPlan {
        fail_after: 1,
        torn_prefix: Some(2),
    });
    assert!(store.write(BlockId(1), 1, b"new").is_err());
    let mut backend = store.into_inner();
    backend.disarm();
    let recovered: JournalStore<Faulty, 8> = JournalStore::mount(backend).unwrap();
    let mut bytes = [0u8; 3];
    recovered.read(BlockId(1), &mut bytes).unwrap();
    assert!(bytes == *b"old" || bytes == *b"new");
}

#[test]
fn f08_getting_started_workflows_are_all_automated() {
    let results = run_getting_started_workflows();
    assert_eq!(results.len(), 10);
    assert!(results.iter().all(|result| result.passed), "{results:#?}");
}

#[test]
fn f08_can_demo_echo_and_counter_use_injected_time() {
    let input = CanFrame::with_data(CanId::base(0x124), &[1, 2]);
    let echo = CanDemo::echo(&input).unwrap();
    assert_eq!(echo.id().raw_id(), 0x125);
    assert_eq!(echo.payload(), &[1, 2]);
    let mut demo = CanDemo::new(Instant::from_nanos(1_000));
    assert!(demo.poll_counter(Instant::from_nanos(999)).is_none());
    assert_eq!(
        demo.poll_counter(Instant::from_nanos(1_000))
            .unwrap()
            .payload(),
        &[0, 0, 0, 0]
    );
    assert!(demo
        .poll_counter(Instant::from_nanos(1_000).wrapping_add(Duration::from_nanos(999_999_999)))
        .is_none());
}

#[test]
fn f09_oracle_output_is_deterministic_and_scrubbed() {
    let first = oracle_json("rust");
    let second = oracle_json("rust");
    assert_eq!(first, second);
    assert!(first.contains("ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77"));
    assert!(!first.contains("H:\\"));
}

#[test]
fn f10_client_capacity_is_bounded_and_recovers() {
    let mut limiter: ClientLimiter<3> = ClientLimiter::new();
    let tokens = [
        limiter.try_open().unwrap(),
        limiter.try_open().unwrap(),
        limiter.try_open().unwrap(),
    ];
    assert_eq!(limiter.active(), 3);
    assert_eq!(limiter.try_open(), None);
    assert_eq!(limiter.rejected(), 1);
    assert!(limiter.close(tokens[1]));
    assert!(limiter.try_open().is_some());
}

#[test]
fn f10_malformed_traffic_is_panic_free_and_bounded() {
    let mut core = DiagnosticCore::new();
    for length in 0..=512 {
        let bytes = vec![0xff; length];
        let response = core.dispatch_at(
            DiagnosticTransport::DoIp,
            &bytes,
            Instant::from_nanos(length as u64),
        );
        assert!(response.bytes().len() <= 128);
    }
}

#[test]
fn f10_concurrent_clients_share_state_without_loss() {
    let core = Arc::new(Mutex::new(DiagnosticCore::new()));
    let mut workers = Vec::new();
    for worker in 0..8 {
        let core = Arc::clone(&core);
        workers.push(thread::spawn(move || {
            for request in 0..250 {
                core.lock().unwrap().dispatch_at(
                    DiagnosticTransport::DoIp,
                    &[0x3e, 0x00],
                    Instant::from_nanos(worker * 250 + request),
                );
            }
        }));
    }
    for worker in workers {
        worker.join().unwrap();
    }
    assert_eq!(core.lock().unwrap().dispatch_count(), 2_000);
}

#[test]
fn f10_bounded_soak_and_restarts_do_not_leak_state() {
    let mut app = ReferenceApp::new(AppConfig::default()).unwrap();
    for cycle in 0..10 {
        let now = Instant::from_nanos(cycle * 1_000_000);
        app.start(now).unwrap();
        for _ in 0..1_000 {
            assert!(app.command("stats all", now).starts_with("commands="));
        }
        app.shutdown(now).unwrap();
        assert_eq!(app.level(), 0);
    }
}
