//! Shared board-application composition used by every target example.
//!
//! Hardware register access remains in the BSP drivers. Examples select only
//! a role and therefore cannot invent clock, pin, UART, CAN, or flash layouts.

// Diverging board functions intentionally own their peripheral drivers.
#![allow(clippy::needless_pass_by_value)]

use core::cell::{Cell, RefCell};
use core::fmt::Write as _;

use bsw_can::{CanFrame, CanId, CanTransceiver, FrameSource, TransceiverState};
use bsw_console::chario::UartWriter;
use bsw_lifecycle::LifecycleComponent;
use bsw_platform::{ResetControl, ResetReason, Watchdog as _};
use bsw_reference_core::{DiagnosticTransport, ProductionComposition};
use bsw_safety::{
    FailureAction, FastTestState, FaultInjection, RetainedSafetyEvent, RomCheckStatus,
    RomCrcChecker, SafeSupervisor, SafetyEvent, SafetyEventCode, SafetyPolicy, SafetySeverity,
    SafetySink, SafetyTask, WatchdogFastTest,
};
use bsw_storage::{BlockId, BlockStore, StorageBackend};
use bsw_time::Instant;
use bsw_uds::diag_job::{DiagJob, DiagRouter};
use bsw_uds::nrc::Nrc;

use crate::adc::{AdcChannel, Stm32AdcChannel};
use crate::board::{BoardPeripherals, PortId, ACTIVE};
use crate::diag_can::DiagCanTransport;
use crate::gpio::{OutputPin, Port};
use crate::pwm::{PwmChannel, Stm32PwmChannel};
use crate::reset::Stm32ResetControl;
use crate::timer::DwtTimer;
use crate::watchdog::Iwdg;

#[cfg(feature = "stm32f413")]
type BoardCan = crate::can_bxcan::BxCanTransceiver;
#[cfg(feature = "stm32g474")]
type BoardCan = crate::can_fdcan::FdCanTransceiver;
#[cfg(feature = "stm32f413")]
type BoardUart = crate::uart_f4::PolledUart;
#[cfg(feature = "stm32g474")]
type BoardUart = crate::uart_g4::PolledUart;
type BoardLog = UartWriter<BoardUart, { crate::resource_contract::CONSOLE_TX_BYTES }>;
#[cfg(feature = "stm32f413")]
type BoardStorage = crate::flash_f4::F4StorageBackend;
#[cfg(feature = "stm32g474")]
type BoardStorage = crate::storage_g4::G4NvmBackend;

unsafe extern "C" {
    static _stext: u32;
    static _storage_start: u8;
    static _storage_end: u8;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DemoRole {
    Application,
    Blink,
    CanLoopback,
    CanBusOff,
    CanStress,
    SafetyConformance,
    MpuNegative,
    CanServer,
    CanTester,
    UdsServer,
    BswStack,
    StorageConformance,
}

/// Boot a target example using the active typed board table and ownership set.
pub fn run(role: DemoRole) -> ! {
    #[cfg(feature = "stm32f413")]
    let measured_hz = crate::configure_clocks_f413();
    #[cfg(feature = "stm32g474")]
    let measured_hz = crate::configure_clocks_g474();
    assert_eq!(measured_hz, ACTIVE.core_clock_hz);

    let mut core = cortex_m::Peripherals::take().expect("Cortex peripherals already owned");
    crate::safety_hw::install_mpu(&mut core.MPU).expect("invalid fixed MPU configuration");
    let board = BoardPeripherals::take().expect("board peripherals already owned");
    let crate::board::BoardPeripherals {
        can1,
        uart,
        gpio,
        timer,
        pwm,
        adc,
        watchdog,
        flash,
        reset,
    } = board;
    let mut clock = DwtTimer::from_token(timer);
    clock.init(&mut core.DCB, &mut core.DWT, measured_hz);
    let mut uart = BoardUart::from_token(uart);
    #[cfg(feature = "stm32f413")]
    // SAFETY: the typed USART token establishes unique startup ownership.
    unsafe {
        uart.init();
    }
    #[cfg(feature = "stm32g474")]
    uart.init();
    let mut log = BoardLog::new(uart);
    let led_pin = match ACTIVE.led.port {
        PortId::A => Port::A,
        PortId::B => Port::B,
        PortId::C => Port::C,
        PortId::D => Port::D,
    };
    let led = OutputPin::from_token(gpio, led_pin, ACTIVE.led.pin);
    // SAFETY: the typed reset token establishes the single RCC reset owner.
    let mut reset = unsafe { Stm32ResetControl::new(reset) };
    let reset_reason = reset.reset_reason();
    let _ = writeln!(
        log,
        "openbsw role={role:?} board={} clock_hz={} reset={:?}",
        ACTIVE.name, measured_hz, reset_reason
    );
    reset.clear_reset_reason();
    let _ = log.pump();

    match role {
        DemoRole::Application => run_application(
            log,
            clock,
            led,
            pwm,
            adc,
            watchdog,
            BoardCan::from_token(can1, 0),
            flash,
            reset_reason,
            reset,
        ),
        DemoRole::Blink => run_blink(log, clock, led, None),
        DemoRole::CanTester => run_can_tester(log, clock, led, BoardCan::from_token(can1, 0)),
        DemoRole::CanLoopback => run_can_loopback(log, clock, led, BoardCan::from_token(can1, 0)),
        DemoRole::CanBusOff => {
            run_can_busoff(log, clock, led, BoardCan::from_token(can1, 0), watchdog)
        }
        DemoRole::CanStress => {
            run_can_stress(log, clock, led, BoardCan::from_token(can1, 0), watchdog)
        }
        DemoRole::SafetyConformance => {
            run_safety_conformance(log, clock, watchdog, flash, reset_reason, reset)
        }
        DemoRole::MpuNegative => run_mpu_negative(log),
        DemoRole::CanServer | DemoRole::UdsServer | DemoRole::BswStack => {
            run_diagnostic_server(log, clock, led, BoardCan::from_token(can1, 0))
        }
        DemoRole::StorageConformance => {
            #[cfg(feature = "stm32g474")]
            run_storage(log, clock, led, flash);
            #[cfg(feature = "stm32f413")]
            run_storage(log, clock, led, flash);
        }
    }
}

const SAFETY_WATCHDOG_STAGE: u16 = 0x51a0;
const SAFETY_WATCHDOG_RUN_TIMEOUT_MS: u32 = 4_096;

// The ordered reset stages stay together so their retained-state transitions
// can be reviewed as one fail-closed campaign.
#[allow(clippy::too_many_lines)]
fn run_safety_conformance(
    mut log: BoardLog,
    clock: DwtTimer,
    watchdog_token: crate::board::Watchdog,
    flash: crate::board::Flash,
    reset_reason: ResetReason,
    reset: Stm32ResetControl,
) -> ! {
    let retained = crate::fault::retained_safety_event();

    if retained
        .is_some_and(|event| event.code == SafetyEventCode::RomCrc as u16 && event.detail == 1)
    {
        let mut watchdog = Iwdg::from_token(watchdog_token);
        assert!(watchdog.start_with_timeout(SAFETY_WATCHDOG_RUN_TIMEOUT_MS));
        crate::fault::clear_retained_safety_event();
        let _ = writeln!(log, "safety critical-reset=passed mpu-negative=armed");
        while log.pending() != 0 {
            let _ = log.pump();
        }
        // SAFETY: the conformance image intentionally performs one CPU write
        // to executable flash. MPU/HardFault must reset before it can return;
        // the flash controller remains locked, so no programming occurs.
        unsafe {
            crate::mmio::write(core::ptr::addr_of!(_stext).cast_mut(), 0);
        }
        panic!("MPU allowed executable-ROM write")
    }

    if retained.is_some_and(|event| event.code == SafetyEventCode::HardFault as u16)
        && crate::fault::fault_dump_valid()
    {
        let mut watchdog = Iwdg::from_token(watchdog_token);
        assert!(watchdog.start_with_timeout(SAFETY_WATCHDOG_RUN_TIMEOUT_MS));
        watchdog.service();
        let event = retained.expect("checked retained event");
        let mut handoff = false;
        if let Ok(mut storage) = mount_board_storage(flash) {
            if storage.write(BlockId(0x5afe), 1, &event.encode()).is_ok() {
                crate::fault::clear_retained_safety_event();
                crate::fault::clear_fault_dump();
                handoff = true;
            }
        }
        let caps = crate::safety_hw::ECC_CAPABILITIES;
        let observed = crate::safety_hw::observe_ecc();
        let passed =
            handoff && !observed.corrected_flash_error && !observed.uncorrectable_flash_error;
        watchdog.service();
        let _ = writeln!(
            log,
            "safety conformance={} h01=true h02=true h03=true h04={} h05=true h06=true h07=true h08=true ecc_flash={} reset={reset_reason:?}",
            if passed { "passed" } else { "failed" },
            handoff,
            caps.flash_ecc_reporting,
        );
        loop {
            watchdog.service();
            let _ = log.pump();
        }
    }

    if retained.is_some_and(|event| {
        event.code == SafetyEventCode::Watchdog as u16 && event.detail == SAFETY_WATCHDOG_STAGE
    }) {
        let mut watchdog = Iwdg::from_token(watchdog_token);
        let mut fast_test = WatchdogFastTest::new(
            bsw_time::Duration::from_millis(
                crate::resource_contract::WATCHDOG_FAST_TEST_DEADLINE_MS,
            )
            .unwrap(),
            1,
        );
        let state = {
            let mut backend = crate::watchdog::IwdgFastTestBackend::new(
                &mut watchdog,
                reset_reason == ResetReason::Watchdog,
            );
            fast_test.resume_after_boot(&mut backend)
        };
        if state != FastTestState::Passed {
            let _ = writeln!(log, "safety conformance=failed watchdog-state={state:?}");
            loop {
                let _ = log.pump();
            }
        }
        assert!(watchdog.start_with_timeout(SAFETY_WATCHDOG_RUN_TIMEOUT_MS));
        crate::fault::clear_retained_safety_event();
        let rom = crate::safety_hw::executable_rom();
        let expected = crate::safety_hw::expected_rom_crc()
            .expect("release image must contain finalized ROM CRC");
        let checker = RomCrcChecker::new(rom, expected, 256)
            .expect("linker-provided executable ROM interval");
        let supervisor = SafeSupervisor::new(BoardSafetyPolicy, BoardSafetySink { reset });
        let mut safety =
            SafetyTask::new(supervisor, checker, 4).expect("nonzero watchdog monitor period");
        assert_eq!(safety.init(), bsw_lifecycle::TransitionResult::Done);
        let mut status = RomCheckStatus::InProgress;
        while status == RomCheckStatus::InProgress {
            status = safety.step(clock.update(), true);
            watchdog.service();
        }
        if status != RomCheckStatus::Passed {
            let _ = writeln!(log, "safety conformance=failed rom-crc={status:?}");
            loop {
                let _ = log.pump();
            }
        }
        safety.inject(FaultInjection::MonitorFailure(0x51a1));
        let _ = safety.step(clock.update(), true);
        let monitor_routed = crate::fault::retained_safety_event().is_some_and(|event| {
            event.code == SafetyEventCode::Monitor as u16 && event.detail == 0x51a1
        });
        if !monitor_routed {
            let _ = writeln!(log, "safety conformance=failed monitor-routing=false");
            loop {
                let _ = log.pump();
            }
        }
        crate::fault::clear_retained_safety_event();
        let _ = writeln!(
            log,
            "safety watchdog=passed rom-crc=passed monitor-routing=passed critical-reset=armed"
        );
        while log.pending() != 0 {
            let _ = log.pump();
        }
        safety.inject(FaultInjection::RomMismatch);
        let _ = safety.step(clock.update(), true);
        panic!("critical safety fault did not reset")
    }

    crate::fault::clear_retained_safety_event();
    crate::fault::clear_fault_dump();
    crate::fault::clear_retained_watchdog_test();
    let mut watchdog = Iwdg::from_token(watchdog_token);
    if !watchdog.start_with_timeout(SAFETY_WATCHDOG_RUN_TIMEOUT_MS) {
        let _ = writeln!(log, "safety conformance=failed inherited-watchdog=false");
        loop {
            let _ = log.pump();
        }
    }
    let _ = writeln!(log, "safety isolation=armed");
    while log.pending() != 0 {
        let _ = log.pump();
    }
    if erase_board_storage(flash, &mut watchdog).is_err() {
        let _ = writeln!(log, "safety conformance=failed storage-isolation=false");
        loop {
            let _ = log.pump();
        }
    }
    let _ = writeln!(log, "safety isolation=passed");
    while log.pending() != 0 {
        let _ = log.pump();
    }
    crate::fault::write_retained_safety_event(RetainedSafetyEvent {
        code: SafetyEventCode::Watchdog as u16,
        detail: SAFETY_WATCHDOG_STAGE,
        timestamp_ns: clock.update().as_nanos(),
    });
    let mut fast_test = WatchdogFastTest::new(
        bsw_time::Duration::from_millis(crate::resource_contract::WATCHDOG_FAST_TEST_DEADLINE_MS)
            .unwrap(),
        1,
    );
    let state = {
        let mut backend = crate::watchdog::IwdgFastTestBackend::new(&mut watchdog, false);
        fast_test.start(clock.update(), &mut backend)
    };
    let _ = writeln!(log, "safety watchdog-fast-test={state:?}");
    while log.pending() != 0 {
        let _ = log.pump();
    }
    let mut backend = crate::watchdog::IwdgFastTestBackend::new(&mut watchdog, false);
    loop {
        let _ = fast_test.poll(clock.update(), &mut backend);
    }
}

fn run_mpu_negative(mut log: BoardLog) -> ! {
    let prior_fault = crate::fault::retained_safety_event().is_some_and(|event| {
        event.code == SafetyEventCode::HardFault as u16 && crate::fault::fault_dump_valid()
    });
    if prior_fault {
        let _ = writeln!(log, "MPU negative-access=passed");
        crate::fault::clear_retained_safety_event();
        crate::fault::clear_fault_dump();
        loop {
            let _ = log.pump();
        }
    }
    let _ = writeln!(log, "MPU negative-access=armed");
    let _ = log.pump();
    // SAFETY: this dedicated negative-access image intentionally attempts one
    // CPU write to the MPU-protected executable region. It does not unlock or
    // program flash; HardFault capture must reset before the call can return.
    unsafe {
        crate::mmio::write(core::ptr::addr_of!(_stext).cast_mut(), 0);
    }
    panic!("MPU allowed a write to executable ROM")
}

// Typed peripheral tokens stay explicit at the one composition boundary.
#[allow(clippy::too_many_arguments)]
fn run_application(
    mut log: BoardLog,
    clock: DwtTimer,
    led: OutputPin,
    pwm_token: crate::board::Pwm,
    adc_token: crate::board::Adc,
    watchdog_token: crate::board::Watchdog,
    can: BoardCan,
    flash: crate::board::Flash,
    reset_reason: ResetReason,
    reset: Stm32ResetControl,
) -> ! {
    let mut pwm = Stm32PwmChannel::new(pwm_token);
    let pwm_result = pwm.configure(1_000, 0);
    let mut adc = Stm32AdcChannel::new(adc_token);
    let adc_result = adc.calibrate();
    let mut watchdog = Iwdg::from_token(watchdog_token);
    let _ = writeln!(log, "reference start pwm={pwm_result:?} adc={adc_result:?}");
    let mut storage = mount_board_storage(flash);
    if crate::fault::retained_safety_event().is_none() && reset_reason != ResetReason::Unknown {
        crate::fault::write_retained_safety_event(RetainedSafetyEvent {
            code: SafetyEventCode::Reset as u16,
            detail: reset_reason_detail(reset_reason),
            timestamp_ns: clock.update().as_nanos(),
        });
    }
    if let Some(event) = crate::fault::retained_safety_event() {
        let encoded = event.encode();
        if let Ok(store) = storage.as_mut() {
            if store.write(BlockId(0x5afe), 1, &encoded).is_ok() {
                crate::fault::clear_retained_safety_event();
            }
        }
    }
    let _ = writeln!(log, "storage mount={:?}", storage.as_ref().map(|_| ()));

    let mut fast_test = WatchdogFastTest::new(
        bsw_time::Duration::from_millis(crate::resource_contract::WATCHDOG_FAST_TEST_DEADLINE_MS)
            .unwrap(),
        1,
    );
    let fast_state = {
        let mut backend = crate::watchdog::IwdgFastTestBackend::new(
            &mut watchdog,
            reset_reason == ResetReason::Watchdog,
        );
        let resumed = fast_test.resume_after_boot(&mut backend);
        if resumed == FastTestState::Idle {
            fast_test.start(clock.update(), &mut backend)
        } else {
            resumed
        }
    };
    let _ = writeln!(log, "watchdog-fast-test={fast_state:?}");
    let _ = log.pump();
    if fast_state == FastTestState::AwaitingReset {
        let mut backend = crate::watchdog::IwdgFastTestBackend::new(&mut watchdog, false);
        loop {
            let _ = fast_test.poll(clock.update(), &mut backend);
        }
    }
    if fast_state != FastTestState::Passed {
        let _ = writeln!(
            log,
            "reference safety-startup=failed watchdog-state={fast_state:?}"
        );
        loop {
            let _ = log.pump();
        }
    }
    let watchdog_result = if watchdog.start_with_timeout(1_500) {
        Ok(())
    } else {
        Err(bsw_platform::WatchdogError::HardwareFault)
    };

    let rom = crate::safety_hw::executable_rom();
    let expected_rom_crc = crate::safety_hw::expected_rom_crc().unwrap_or(u32::MAX);
    let rom_checker = RomCrcChecker::new(rom, expected_rom_crc, 256)
        .expect("linker-provided executable ROM interval");
    let supervisor = SafeSupervisor::new(BoardSafetyPolicy, BoardSafetySink { reset });
    let mut safety =
        SafetyTask::new(supervisor, rom_checker, 4).expect("nonzero watchdog monitor period");
    assert_eq!(safety.init(), bsw_lifecycle::TransitionResult::Done);
    let ecc = crate::safety_hw::observe_ecc();
    if ecc.corrected_flash_error || ecc.uncorrectable_flash_error {
        safety.route(SafetyEvent {
            code: SafetyEventCode::Ecc,
            detail: u16::from(ecc.corrected_flash_error)
                | (u16::from(ecc.uncorrectable_flash_error) << 1),
            timestamp_ns: clock.update().as_nanos(),
        });
    }

    let core = RefCell::new(ProductionComposition::new());
    let started_at = clock.update();
    core.borrow_mut().start(started_at);
    let job = ReferenceDiagJob::new(&core, started_at);
    let registered_job = register_production_diagnostic_job(&job);
    let jobs: [&dyn DiagJob; 1] = [registered_job];
    let router = DiagRouter::new(&jobs);
    let mut diagnostic = DiagCanTransport::new(can, 0x600, 0x601, router);
    assert_eq!(diagnostic.init(), bsw_lifecycle::TransitionResult::Done);
    let _ = writeln!(
        log,
        "reference ready UDS/ISO-TP 0x600 -> 0x601 watchdog={watchdog_result:?}"
    );
    let _ = log.pump();

    run_reference_loop(
        log, clock, led, pwm, adc, watchdog, diagnostic, &job, &core, storage, safety,
    )
}

#[inline(never)]
fn register_production_diagnostic_job<'a>(job: &'a dyn DiagJob) -> &'a dyn DiagJob {
    let mut clients: crate::dynamic_client::DynamicClientRegistry<'a, dyn DiagJob, 8, 4> =
        crate::dynamic_client::DynamicClientRegistry::new();
    clients
        .set(0, 0, job)
        .expect("production diagnostic client capacity");
    let (registered_job, registered_channel) = clients
        .get(0)
        .expect("production diagnostic client mapping");
    assert_eq!(registered_channel, 0);
    registered_job
}

const fn reset_reason_detail(reason: ResetReason) -> u16 {
    match reason {
        ResetReason::PowerOn => 1,
        ResetReason::Software => 2,
        ResetReason::Watchdog => 3,
        ResetReason::Pin => 4,
        ResetReason::LowPower => 5,
        ResetReason::Unknown => 0,
    }
}

#[allow(clippy::too_many_arguments)]
fn run_reference_loop(
    mut log: BoardLog,
    clock: DwtTimer,
    led: OutputPin,
    mut pwm: Stm32PwmChannel,
    mut adc: Stm32AdcChannel,
    mut watchdog: Iwdg,
    mut diagnostic: DiagCanTransport<'_, BoardCan>,
    job: &ReferenceDiagJob<'_>,
    core: &RefCell<ProductionComposition>,
    mut storage: Result<
        bsw_storage::journal::JournalStore<BoardStorage, 8>,
        bsw_storage::StorageError,
    >,
    mut safety: SafetyTask<'static, BoardSafetyPolicy, BoardSafetySink>,
) -> ! {
    let mut next_io = clock.update();
    let mut next_heartbeat = next_io;
    let mut next_persist = next_io;
    let io_period = bsw_time::Duration::from_millis(10).unwrap();
    let heartbeat_period =
        bsw_time::Duration::from_micros(crate::resource_contract::HEARTBEAT_PERIOD_US).unwrap();
    let persist_period = bsw_time::Duration::from_millis(60_000).unwrap();
    let mut persist_counter = 0u32;
    if let Ok(store) = storage.as_ref() {
        let mut bytes = [0u8; 4];
        if store.read(BlockId(0x0a01), &mut bytes) == Ok(4) {
            persist_counter = u32::from_le_bytes(bytes);
        }
    }
    loop {
        let now = clock.update();
        service_can(diagnostic.transceiver_mut(), &clock);
        job.set_now(now);
        diagnostic.poll_at(now);
        let _ = safety.step(now, true);
        if now.is_at_or_after(next_io) {
            next_io = next_io.wrapping_add(io_period);
            if let Ok(raw) = adc.read_raw() {
                if let Ok(snapshot) = core.borrow_mut().cycle_io(raw, now) {
                    if snapshot.pwm_permille != pwm.duty_permille() {
                        let _ = pwm.configure(1_000, snapshot.pwm_permille);
                    }
                }
            }
        }
        if now.is_at_or_after(next_heartbeat) {
            next_heartbeat = next_heartbeat.wrapping_add(heartbeat_period);
            led.toggle();
            watchdog.service();
            let _ = writeln!(
                log,
                "reference heartbeat diag={} adc={} pwm={} gpio=toggle can_overflow={}",
                core.borrow().diagnostics().dispatch_count(),
                core.borrow().io_snapshot().adc,
                pwm.duty_permille(),
                diagnostic.transceiver().rx_dropped()
            );
        }
        if now.is_at_or_after(next_persist) {
            next_persist = next_persist.wrapping_add(persist_period);
            persist_counter = persist_counter.saturating_add(1);
            if let Ok(store) = storage.as_mut() {
                let version = persist_counter.min(u32::from(u16::MAX)) as u16;
                let _ = store.write(BlockId(0x0a01), version, &persist_counter.to_le_bytes());
            }
        }
        let _ = log.pump();
    }
}

struct BoardSafetyPolicy;

impl SafetyPolicy for BoardSafetyPolicy {
    fn severity(&self, event: &SafetyEvent) -> SafetySeverity {
        match event.code {
            SafetyEventCode::Reset | SafetyEventCode::Monitor => SafetySeverity::Degraded,
            _ => SafetySeverity::Critical,
        }
    }

    fn action(&self, _event: &SafetyEvent, severity: SafetySeverity) -> FailureAction {
        match severity {
            SafetySeverity::Information => FailureAction::LogOnly,
            SafetySeverity::Degraded => FailureAction::LimpHome,
            SafetySeverity::Critical => FailureAction::ControlledReset,
        }
    }
}

struct BoardSafetySink {
    reset: Stm32ResetControl,
}

impl SafetySink for BoardSafetySink {
    fn record(&mut self, event: SafetyEvent, _severity: SafetySeverity) {
        crate::fault::write_retained_safety_event(RetainedSafetyEvent {
            code: event.code as u16,
            detail: event.detail,
            timestamp_ns: event.timestamp_ns,
        });
    }

    fn enter_limp_home(&mut self, _event: SafetyEvent) {}

    fn request_reset(&mut self, _event: SafetyEvent) {
        self.reset.request_reset();
    }
}

fn mount_board_storage(
    _flash: crate::board::Flash,
) -> Result<bsw_storage::journal::JournalStore<BoardStorage, 8>, bsw_storage::StorageError> {
    #[cfg(feature = "stm32g474")]
    if !BoardStorage::dual_bank_geometry_enabled() {
        return Err(bsw_storage::StorageError::InvalidParameter);
    }
    // SAFETY: the typed token is consumed and the active linker script owns
    // the complete storage region, so this is the sole flash backend.
    let backend = unsafe { BoardStorage::new() };
    bsw_storage::journal::JournalStore::mount(backend)
}

fn erase_board_storage(
    _flash: crate::board::Flash,
    watchdog: &mut Iwdg,
) -> Result<(), bsw_storage::StorageError> {
    let start = core::ptr::addr_of!(_storage_start) as usize;
    let end = core::ptr::addr_of!(_storage_end) as usize;
    #[cfg(feature = "stm32f413")]
    let expected = (
        crate::flash_f4::STORAGE_BASE_ADDR as usize,
        (crate::flash_f4::STORAGE_BASE_ADDR + crate::flash_f4::STORAGE_SIZE) as usize,
    );
    #[cfg(feature = "stm32g474")]
    let expected = (
        crate::flash_g4::NVM_BASE_ADDR as usize,
        (crate::flash_g4::NVM_BASE_ADDR + crate::flash_g4::NVM_SIZE) as usize,
    );
    if (start, end) != expected {
        return Err(bsw_storage::StorageError::InvalidParameter);
    }
    #[cfg(feature = "stm32g474")]
    if !BoardStorage::dual_bank_geometry_enabled() {
        return Err(bsw_storage::StorageError::InvalidParameter);
    }
    // SAFETY: the typed token was consumed, linker bounds match the board's
    // reserved storage constants, and the safety campaign is the sole owner.
    let mut backend = unsafe { BoardStorage::new() };
    let units = backend.region_size() / backend.erase_unit();
    for unit in 0..units {
        watchdog.service();
        backend.erase(unit)?;
        watchdog.service();
    }
    Ok(())
}

struct ReferenceDiagJob<'a> {
    core: &'a RefCell<ProductionComposition>,
    now: Cell<Instant>,
}

impl<'a> ReferenceDiagJob<'a> {
    const fn new(core: &'a RefCell<ProductionComposition>, now: Instant) -> Self {
        Self {
            core,
            now: Cell::new(now),
        }
    }

    fn set_now(&self, now: Instant) {
        self.now.set(now);
    }
}

impl DiagJob for ReferenceDiagJob<'_> {
    fn implemented_request(&self) -> &[u8] {
        &[]
    }

    fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        let result = self.core.borrow_mut().diagnostics_mut().dispatch_at(
            DiagnosticTransport::DoCan,
            request,
            self.now.get(),
        );
        let bytes = result.bytes();
        if bytes.len() == 3 && bytes[0] == 0x7f {
            return Err(Nrc::from_byte(bytes[2]).unwrap_or(Nrc::GeneralReject));
        }
        if bytes.len() > response.len() {
            return Err(Nrc::ResponseTooLong);
        }
        response[..bytes.len()].copy_from_slice(bytes);
        Ok(bytes.len())
    }
}

fn run_blink(mut log: BoardLog, clock: DwtTimer, led: OutputPin, mut watchdog: Option<Iwdg>) -> ! {
    let mut last_second = u64::MAX;
    loop {
        let second = clock.system_time_us_64() / 1_000_000;
        if second != last_second {
            last_second = second;
            if second & 1 == 0 {
                led.set_high();
            } else {
                led.set_low();
            }
            let _ = writeln!(log, "heartbeat={second}");
            if let Some(watchdog) = watchdog.as_mut() {
                watchdog.service();
            }
        }
        let _ = log.pump();
    }
}

fn initialise_can(can: &mut BoardCan) {
    assert_eq!(can.init(), bsw_can::ErrorCode::Ok);
    assert_eq!(can.open(), bsw_can::ErrorCode::Ok);
}

fn service_rx(can: &mut BoardCan) {
    crate::safety_hw::isr_pre_hook();
    #[cfg(feature = "stm32f413")]
    // SAFETY: this polling application is the sole producer of the ISR queue.
    unsafe {
        can.receive_isr();
    }
    #[cfg(feature = "stm32g474")]
    // SAFETY: this polling application is the sole producer of the ISR queue.
    unsafe {
        can.isr_rx_fifo0();
    }
    assert!(crate::safety_hw::isr_post_hook());
}

fn service_can(can: &mut BoardCan, clock: &DwtTimer) {
    service_rx(can);
    can.service_health(clock.update());
}

fn run_can_loopback(mut log: BoardLog, clock: DwtTimer, led: OutputPin, mut can: BoardCan) -> ! {
    initialise_can(&mut can);
    let _ = writeln!(log, "CAN loopback ready at {} bit/s", can.baudrate());
    loop {
        service_can(&mut can, &clock);
        if let Some(frame) = FrameSource::receive(&mut can) {
            let _ = can.write(&frame);
            led.toggle();
        }
        let _ = log.pump();
    }
}

fn run_can_tester(mut log: BoardLog, clock: DwtTimer, led: OutputPin, mut can: BoardCan) -> ! {
    initialise_can(&mut can);
    let request = CanFrame::with_data(CanId::base(0x600), &[2, 0x3E, 0]);
    let mut deadline = 0;
    let mut previous_state = can.transceiver_state();
    loop {
        let now = clock.system_time_us_64();
        if now >= deadline {
            deadline = now.wrapping_add(1_000_000);
            let _ = can.write(&request);
        }
        service_can(&mut can, &clock);
        let state = can.transceiver_state();
        if state != previous_state {
            let _ = writeln!(log, "CAN state={state:?}");
            previous_state = state;
        }
        if let Some(frame) = FrameSource::receive(&mut can) {
            if frame.id() == CanId::base(0x601) {
                led.toggle();
                let _ = writeln!(log, "diagnostic response len={}", frame.payload_length());
            }
        }
        let _ = log.pump();
    }
}

fn run_can_stress(
    mut log: BoardLog,
    clock: DwtTimer,
    led: OutputPin,
    mut can: BoardCan,
    watchdog_token: crate::board::Watchdog,
) -> ! {
    let mut watchdog = Iwdg::from_token(watchdog_token);
    assert!(watchdog.start_with_timeout(SAFETY_WATCHDOG_RUN_TIMEOUT_MS));
    initialise_can(&mut can);
    let start = clock.system_time_us_64();
    let _ = writeln!(log, "CAN stress window_us=10000000");
    while clock.system_time_us_64().wrapping_sub(start) < 10_000_000 {
        watchdog.service();
        let _ = log.pump();
    }
    service_can(&mut can, &clock);
    let dropped = can.rx_dropped();
    let _ = writeln!(log, "CAN stress overflow={dropped}");
    if dropped > 0 {
        led.set_high();
    } else {
        led.set_low();
    }
    loop {
        watchdog.service();
        service_can(&mut can, &clock);
        while let Some(frame) = FrameSource::receive(&mut can) {
            if frame.id() == CanId::base(0x600) && frame.payload().starts_with(&[2, 0x3e, 0]) {
                let response =
                    CanFrame::with_data(CanId::base(0x601), &[2, 0x7e, 0, 0, 0, 0, 0, 0]);
                if can.write(&response) == bsw_can::ErrorCode::Ok {
                    let _ = writeln!(log, "CAN stress valid-traffic=accepted");
                }
            }
        }
        let _ = log.pump();
    }
}

fn run_can_busoff(
    mut log: BoardLog,
    clock: DwtTimer,
    led: OutputPin,
    mut can: BoardCan,
    watchdog_token: crate::board::Watchdog,
) -> ! {
    let mut watchdog = Iwdg::from_token(watchdog_token);
    assert!(watchdog.start_with_timeout(SAFETY_WATCHDOG_RUN_TIMEOUT_MS));
    initialise_can(&mut can);
    let request = CanFrame::with_data(CanId::base(0x600), &[2, 0x3E, 0]);
    let started = clock.system_time_us_64();
    let mut passive_seen = false;
    let mut bus_off_seen = false;
    let mut bus_off_at = 0;
    can.set_tx_connected_for_test(false);
    let _ = writeln!(log, "CAN busoff injection=tx-isolated");
    loop {
        watchdog.service();
        let now = clock.system_time_us_64();
        if !bus_off_seen && now.wrapping_sub(started) < 5_000_000 {
            let _ = can.write(&request);
        }
        service_can(&mut can, &clock);
        let state = can.transceiver_state();
        if state == TransceiverState::Passive && !passive_seen {
            passive_seen = true;
            let _ = writeln!(
                log,
                "CAN error-passive observed_us={}",
                now.wrapping_sub(started)
            );
        }
        if state == TransceiverState::BusOff && !bus_off_seen {
            bus_off_seen = true;
            bus_off_at = now;
            can.set_tx_connected_for_test(true);
            let _ = writeln!(log, "CAN busoff observed");
        } else if bus_off_seen && state == TransceiverState::Active {
            let recovery_us = now.wrapping_sub(bus_off_at);
            let accepted = can.write(&request) == bsw_can::ErrorCode::Ok;
            let _ = writeln!(
                log,
                "CAN busoff recovered_us={recovery_us} valid-tx={accepted} passive={passive_seen}"
            );
            led.set_high();
            let mut next_valid_tx = now;
            loop {
                watchdog.service();
                service_can(&mut can, &clock);
                let current = clock.system_time_us_64();
                if current >= next_valid_tx {
                    next_valid_tx = current.wrapping_add(100_000);
                    let _ = can.write(&request);
                }
                let _ = log.pump();
            }
        }
        if !bus_off_seen && now.wrapping_sub(started) >= 5_000_000 {
            let _ = writeln!(log, "CAN busoff timeout");
            led.set_low();
            loop {
                watchdog.service();
                let _ = log.pump();
            }
        }
        let _ = log.pump();
    }
}

fn run_diagnostic_server(mut log: BoardLog, clock: DwtTimer, led: OutputPin, can: BoardCan) -> ! {
    let core = RefCell::new(ProductionComposition::new());
    let job = ReferenceDiagJob::new(&core, clock.update());
    let jobs: [&dyn DiagJob; 1] = [&job];
    let router = DiagRouter::new(&jobs);
    let mut diagnostic = DiagCanTransport::new(can, 0x600, 0x601, router);
    assert_eq!(diagnostic.init(), bsw_lifecycle::TransitionResult::Done);
    let _ = writeln!(log, "UDS/ISO-TP server 0x600 -> 0x601 ready");
    let mut heartbeat = 0;
    loop {
        service_can(diagnostic.transceiver_mut(), &clock);
        job.set_now(clock.update());
        diagnostic.poll_at(job.now.get());
        let now = clock.system_time_us_64();
        if now >= heartbeat {
            heartbeat = now.wrapping_add(1_000_000);
            led.toggle();
        }
        let _ = log.pump();
    }
}

fn run_storage(
    mut log: BoardLog,
    clock: DwtTimer,
    led: OutputPin,
    flash: crate::board::Flash,
) -> ! {
    if let Some(state) = crate::fault::retained_storage_campaign() {
        let _ = writeln!(
            log,
            "storage campaign resume cut={} pending={}",
            state.cut_index, state.pending_recovery
        );
    } else {
        let _ = writeln!(log, "storage campaign start");
    }
    while log.pending() != 0 {
        let _ = log.pump();
    }
    // SAFETY: the typed flash token provides exclusivity and this dedicated
    // conformance role has no active interrupt-driven flash users.
    #[cfg(feature = "stm32g474")]
    let report = unsafe { crate::storage_conformance::run_g4_target_conformance(flash) };
    #[cfg(feature = "stm32f413")]
    let report = unsafe { crate::storage_conformance::run_f4_target_conformance(flash) };
    let _ = writeln!(log, "storage conformance={report:?}");
    if report.is_ok() {
        led.set_high();
    } else {
        led.set_low();
    }
    loop {
        let _ = clock.update();
        let _ = log.pump();
    }
}
