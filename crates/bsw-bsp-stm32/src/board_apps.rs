//! Shared board-application composition used by every target example.
//!
//! Hardware register access remains in the BSP drivers. Examples select only
//! a role and therefore cannot invent clock, pin, UART, CAN, or flash layouts.

// Diverging board functions intentionally own their peripheral drivers.
#![allow(clippy::needless_pass_by_value)]

use core::fmt::Write as _;

use bsw_can::{CanFrame, CanId, CanTransceiver, FrameSource};
use bsw_console::chario::UartWriter;
use bsw_lifecycle::LifecycleComponent;
use bsw_platform::{ResetControl, Watchdog as _};
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
type BoardLog = UartWriter<BoardUart, 256>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DemoRole {
    Application,
    Blink,
    CanLoopback,
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
    #[cfg(feature = "stm32f413")]
    let _ = flash;

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
    let _ = writeln!(
        log,
        "openbsw role={role:?} board={} reset={:?}",
        ACTIVE.name,
        reset.reset_reason()
    );
    reset.clear_reset_reason();
    let _ = log.pump();

    match role {
        DemoRole::Application => run_application(log, clock, led, pwm, adc, watchdog),
        DemoRole::Blink => run_blink(log, clock, led, None),
        DemoRole::CanTester => run_can_tester(log, clock, led, BoardCan::from_token(can1, 0)),
        DemoRole::CanLoopback => run_can_loopback(log, clock, led, BoardCan::from_token(can1, 0)),
        DemoRole::CanServer | DemoRole::UdsServer | DemoRole::BswStack => {
            run_diagnostic_server(log, clock, led, BoardCan::from_token(can1, 0))
        }
        DemoRole::StorageConformance => {
            #[cfg(feature = "stm32g474")]
            run_storage(log, clock, led, flash);
            #[cfg(feature = "stm32f413")]
            run_blink(log, clock, led, None);
        }
    }
}

fn run_application(
    mut log: BoardLog,
    clock: DwtTimer,
    led: OutputPin,
    pwm_token: crate::board::Pwm,
    adc_token: crate::board::Adc,
    watchdog_token: crate::board::Watchdog,
) -> ! {
    let mut pwm = Stm32PwmChannel::new(pwm_token);
    let pwm_result = pwm.configure(1_000, 500);
    let mut adc = Stm32AdcChannel::new(adc_token);
    let adc_result = adc.calibrate().and_then(|()| adc.read_raw());
    let mut watchdog = Iwdg::from_token(watchdog_token);
    let watchdog_result = bsw_platform::Watchdog::start(
        &mut watchdog,
        bsw_time::Duration::from_millis(1500).unwrap(),
    );
    let _ = writeln!(
        log,
        "io pwm={pwm_result:?} adc={adc_result:?} watchdog={watchdog_result:?}"
    );
    run_blink(log, clock, led, Some(watchdog))
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
    loop {
        let now = clock.system_time_us_64();
        if now >= deadline {
            deadline = now.wrapping_add(1_000_000);
            let _ = can.write(&request);
        }
        service_can(&mut can, &clock);
        if let Some(frame) = FrameSource::receive(&mut can) {
            if frame.id() == CanId::base(0x601) {
                led.toggle();
                let _ = writeln!(log, "diagnostic response len={}", frame.payload_length());
            }
        }
        let _ = log.pump();
    }
}

struct TesterPresent;

impl DiagJob for TesterPresent {
    fn implemented_request(&self) -> &[u8] {
        &[0x3E]
    }

    fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        if request.len() != 2 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }
        if response.len() < 2 {
            return Err(Nrc::ResponseTooLong);
        }
        response[..2].copy_from_slice(&[0x7E, request[1] & 0x7F]);
        Ok(2)
    }
}

struct ReadVin;

impl DiagJob for ReadVin {
    fn implemented_request(&self) -> &[u8] {
        &[0x22]
    }

    fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        const VIN: &[u8; 17] = b"OPENBSW-RUST-0001";
        if request != [0x22, 0xF1, 0x90] {
            return Err(Nrc::RequestOutOfRange);
        }
        if response.len() < 20 {
            return Err(Nrc::ResponseTooLong);
        }
        response[..3].copy_from_slice(&[0x62, 0xF1, 0x90]);
        response[3..20].copy_from_slice(VIN);
        Ok(20)
    }
}

fn run_diagnostic_server(mut log: BoardLog, clock: DwtTimer, led: OutputPin, can: BoardCan) -> ! {
    let tester_present = TesterPresent;
    let read_vin = ReadVin;
    let jobs: [&dyn DiagJob; 2] = [&tester_present, &read_vin];
    let router = DiagRouter::new(&jobs);
    let mut diagnostic = DiagCanTransport::new(can, 0x600, 0x601, router);
    assert_eq!(diagnostic.init(), bsw_lifecycle::TransitionResult::Done);
    let _ = writeln!(log, "UDS/ISO-TP server 0x600 -> 0x601 ready");
    let mut heartbeat = 0;
    loop {
        service_can(diagnostic.transceiver_mut(), &clock);
        diagnostic.poll();
        let now = clock.system_time_us_64();
        if now >= heartbeat {
            heartbeat = now.wrapping_add(1_000_000);
            led.toggle();
        }
        let _ = log.pump();
    }
}

#[cfg(feature = "stm32g474")]
fn run_storage(
    mut log: BoardLog,
    clock: DwtTimer,
    led: OutputPin,
    flash: crate::board::Flash,
) -> ! {
    // SAFETY: the typed flash token provides exclusivity and this dedicated
    // conformance role has no active interrupt-driven flash users.
    let report = unsafe { crate::storage_conformance::run_g4_target_conformance(flash) };
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
