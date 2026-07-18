use std::{cell::RefCell, rc::Rc};

use bsw_time::{Duration, Instant};
use bsw_uds::{
    async_jobs::{AsyncExecutor, AsyncJob, AsyncResult},
    connection::{ConnectionError, ConnectionPool, ConnectionRequest, ConnectionState, Direction},
    dispatcher::{
        AddressingRule, DiagnosticJob, DispatchOutcome, Dispatcher, JobResult, RequestAddressing,
    },
    registries::{
        DidHandler, DidRegistry, IoControlHandler, IoControlRegistry, RoutineHandler,
        RoutineRegistry,
    },
    standard_services::{
        apply_effect, clear_diagnostic_information, read_dtc_information, CoreServices,
        ResetAction, ServiceEffect,
    },
    state::{DiagnosticState, KeyAlgorithm, SecurityResetPolicy, StatePersistence},
    timing::{TimingAction, TimingConfig, TimingState},
    DemManager, DiagSession, Nrc, SessionMask,
};

struct Job {
    prefix: &'static [u8],
    value: u8,
    sessions: SessionMask,
    addressing: AddressingRule,
}

impl DiagnosticJob for Job {
    fn request_prefix(&self) -> &[u8] {
        self.prefix
    }
    fn sessions(&self) -> SessionMask {
        self.sessions
    }
    fn addressing(&self) -> AddressingRule {
        self.addressing
    }
    fn process(&mut self, _request: &[u8], response: &mut [u8]) -> JobResult {
        if response.is_empty() {
            JobResult::Negative(Nrc::ResponseTooLong)
        } else {
            response[0] = self.value;
            JobResult::Positive(1)
        }
    }
}

#[test]
fn dispatcher_uses_job_tree_order_addressing_and_suppression() {
    let mut dispatcher = Dispatcher::<2>::new();
    assert!(dispatcher.register_transport(7));
    let mut parent = Job {
        prefix: &[0x31],
        value: 1,
        sessions: SessionMask::ALL,
        addressing: AddressingRule::PhysicalAndFunctional,
    };
    let mut child = Job {
        prefix: &[0x31, 0x01],
        value: 2,
        sessions: SessionMask::EXTENDED,
        addressing: AddressingRule::PhysicalOnly,
    };
    let mut jobs: [&mut dyn DiagnosticJob; 2] = [&mut parent, &mut child];
    let mut response = [0; 8];
    assert_eq!(
        dispatcher.dispatch(
            7,
            RequestAddressing::Physical,
            DiagSession::Extended,
            &[0x31, 0x01],
            &mut response,
            &mut jobs
        ),
        Ok(DispatchOutcome::Response(1))
    );
    assert_eq!(response[0], 2);
    assert_eq!(
        dispatcher.dispatch(
            7,
            RequestAddressing::Functional,
            DiagSession::Extended,
            &[0x31, 0x01],
            &mut response,
            &mut jobs
        ),
        Ok(DispatchOutcome::Suppressed)
    );
    assert_eq!(
        dispatcher.dispatch(
            7,
            RequestAddressing::Physical,
            DiagSession::Extended,
            &[0x31, 0x81],
            &mut response,
            &mut jobs
        ),
        Ok(DispatchOutcome::Suppressed)
    );
    assert!(dispatcher.unregister_transport(7));
    assert!(dispatcher
        .dispatch(
            7,
            RequestAddressing::Physical,
            DiagSession::Extended,
            &[0x31],
            &mut response,
            &mut jobs
        )
        .is_err());
}

#[test]
fn connection_pool_exhaustion_ownership_cancel_and_stale_release() {
    let mut pool = ConnectionPool::<1, 8, 8>::new();
    let handle = pool
        .acquire(ConnectionRequest {
            direction: Direction::Incoming,
            addressing: RequestAddressing::Physical,
            source_bus: 4,
            source: 0x1111,
            target: 0x2222,
            payload: &[0x3e, 0],
            notify_processed: true,
        })
        .unwrap();
    assert_eq!(pool.in_use(), 1);
    assert_eq!(
        pool.acquire(ConnectionRequest {
            direction: Direction::Outgoing,
            addressing: RequestAddressing::Physical,
            source_bus: 4,
            source: 1,
            target: 2,
            payload: &[],
            notify_processed: false,
        }),
        Err(ConnectionError::Exhausted)
    );
    assert_eq!(pool.advance(handle), Ok(ConnectionState::Processing));
    assert_eq!(pool.advance(handle), Ok(ConnectionState::Sending));
    assert!(pool.get(handle).unwrap().notify_processed);
    pool.cancel(handle).unwrap();
    assert_eq!(pool.get(handle).unwrap().state, ConnectionState::Terminated);
    pool.release(handle).unwrap();
    assert_eq!(pool.release(handle), Err(ConnectionError::InvalidHandle));
}

#[test]
fn injected_timing_hits_exact_p2_p2star_and_s3_boundaries() {
    let ms = |value| Duration::from_millis(value).unwrap();
    let mut timing = TimingState::new(
        TimingConfig {
            p2: ms(50),
            p2_star: ms(200),
            response_pending_interval: ms(50),
            s3: ms(500),
        },
        Instant::from_nanos(0),
    );
    timing.start_request(Instant::from_nanos(0));
    assert_eq!(
        timing.poll(Instant::from_nanos(ms(50).as_nanos() - 1)),
        TimingAction::None
    );
    assert_eq!(
        timing.poll(Instant::from_nanos(ms(50).as_nanos())),
        TimingAction::ResponsePending
    );
    assert_eq!(
        timing.poll(Instant::from_nanos(ms(100).as_nanos())),
        TimingAction::ResponsePending
    );
    assert_eq!(
        timing.poll(Instant::from_nanos(ms(200).as_nanos())),
        TimingAction::RequestTimeout
    );
    timing.activity(Instant::from_nanos(ms(200).as_nanos()));
    assert_eq!(
        timing.poll(Instant::from_nanos(ms(700).as_nanos())),
        TimingAction::SessionTimeout
    );
}

#[derive(Clone, Default)]
struct Persist(Rc<RefCell<Option<(DiagSession, u8)>>>);

impl StatePersistence for Persist {
    fn load(&mut self) -> Option<(DiagSession, u8)> {
        *self.0.borrow()
    }
    fn store(&mut self, session: DiagSession, attempts: u8) -> bool {
        *self.0.borrow_mut() = Some((session, attempts));
        true
    }
}

struct ProjectKey;

impl KeyAlgorithm for ProjectKey {
    fn verify(&mut self, level: u8, seed: &[u8], key: &[u8]) -> bool {
        level == 2 && seed.iter().rev().copied().eq(key.iter().copied())
    }
}

fn core() -> CoreServices<Persist, ProjectKey> {
    let state = DiagnosticState::new(
        Persist::default(),
        ProjectKey,
        2,
        Duration::from_millis(10).unwrap(),
        SecurityResetPolicy::OnSessionChange,
    );
    CoreServices::new(state, 50, 500)
}

#[test]
fn core_services_persist_session_use_project_key_and_defer_reset() {
    let mut core = core();
    let mut response = [0; 32];
    assert_eq!(
        core.process(&[0x10, 0x03], Instant::from_nanos(0), &[], &mut response)
            .unwrap()
            .0,
        6
    );
    assert_eq!(core.state.session(), DiagSession::Extended);
    assert_eq!(
        core.process(
            &[0x27, 0x01],
            Instant::from_nanos(0),
            &[1, 2, 3],
            &mut response
        )
        .unwrap()
        .0,
        5
    );
    assert_eq!(
        core.process(
            &[0x27, 0x02, 3, 2, 1],
            Instant::from_nanos(0),
            &[],
            &mut response
        )
        .unwrap()
        .0,
        2
    );
    assert!(core.state.is_unlocked(2));
    assert_eq!(
        core.process(
            &[0x28, 0x03, 0x03],
            Instant::from_nanos(0),
            &[],
            &mut response
        )
        .unwrap()
        .0,
        2
    );
    assert_eq!(
        core.communication().normal,
        bsw_uds::standard_services::CommunicationMode::Disabled
    );
    assert_eq!(
        core.process(&[0x11, 0x01], Instant::from_nanos(0), &[], &mut response)
            .unwrap()
            .0,
        2
    );
    assert_eq!(core.take_reset(), None);
    core.response_processed();
    assert_eq!(core.take_reset(), Some(ResetAction::Hard));
}

#[test]
fn security_delay_attempt_limit_and_session_reset_are_deterministic() {
    let mut core = core();
    let mut response = [0; 16];
    core.process(&[0x27, 1], Instant::from_nanos(0), &[4, 5], &mut response)
        .unwrap();
    assert_eq!(
        core.process(&[0x27, 2, 0], Instant::from_nanos(0), &[], &mut response),
        Err(Nrc::InvalidKey)
    );
    assert_eq!(
        core.process(&[0x27, 1], Instant::from_nanos(1), &[4, 5], &mut response),
        Err(Nrc::RequiredTimeDelayNotExpired)
    );
    let after_delay = Instant::from_nanos(Duration::from_millis(10).unwrap().as_nanos());
    core.process(&[0x27, 1], after_delay, &[4, 5], &mut response)
        .unwrap();
    assert_eq!(
        core.process(&[0x27, 2, 0], after_delay, &[], &mut response),
        Err(Nrc::ExceededNumberOfAttempts)
    );
}

struct Did {
    id: u16,
    value: [u8; 2],
    writes: usize,
}

impl DidHandler for Did {
    fn did(&self) -> u16 {
        self.id
    }
    fn write_sessions(&self) -> SessionMask {
        SessionMask::EXTENDED
    }
    fn security_level(&self) -> Option<u8> {
        Some(2)
    }
    fn read(&mut self, output: &mut [u8]) -> Result<usize, Nrc> {
        if output.len() < 2 {
            return Err(Nrc::ResponseTooLong);
        }
        output[..2].copy_from_slice(&self.value);
        Ok(2)
    }
    fn write(&mut self, value: &[u8]) -> Result<(), Nrc> {
        let value: [u8; 2] = value
            .try_into()
            .map_err(|_| Nrc::IncorrectMessageLengthOrInvalidFormat)?;
        self.value = value;
        self.writes += 1;
        Ok(())
    }
}

#[test]
fn did_registry_handles_multiple_reads_permissions_and_durable_write_boundary() {
    let mut first = Did {
        id: 0xf190,
        value: *b"AB",
        writes: 0,
    };
    let mut second = Did {
        id: 0xf191,
        value: *b"CD",
        writes: 0,
    };
    let mut response = [0; 16];
    {
        let mut handlers: [&mut dyn DidHandler; 2] = [&mut first, &mut second];
        let mut registry = DidRegistry::new(&mut handlers).unwrap();
        let length = registry
            .read(
                &[0x22, 0xf1, 0x90, 0xf1, 0x91],
                DiagSession::Extended,
                1 << 2,
                &mut response,
            )
            .unwrap();
        assert_eq!(
            &response[..length],
            &[0x62, 0xf1, 0x90, b'A', b'B', 0xf1, 0x91, b'C', b'D']
        );
        assert_eq!(
            registry.write(
                &[0x2e, 0xf1, 0x90, 1, 2],
                DiagSession::Default,
                1 << 2,
                &mut response
            ),
            Err(Nrc::RequestOutOfRange)
        );
        assert_eq!(
            registry.write(
                &[0x2e, 0xf1, 0x90, 1, 2],
                DiagSession::Extended,
                0,
                &mut response
            ),
            Err(Nrc::SecurityAccessDenied)
        );
        assert_eq!(
            registry.write(
                &[0x2e, 0xf1, 0x90, 1, 2],
                DiagSession::Extended,
                1 << 2,
                &mut response
            ),
            Ok(3)
        );
    }
    assert_eq!(first.value, [1, 2]);
    assert_eq!(first.writes, 1);
}

#[derive(Default)]
struct Routine {
    running: bool,
}

impl RoutineHandler for Routine {
    fn routine_id(&self) -> u16 {
        0x1234
    }
    fn start(&mut self, input: &[u8], output: &mut [u8]) -> Result<usize, Nrc> {
        if self.running {
            return Err(Nrc::RequestSequenceError);
        }
        self.running = true;
        output[0] = input.first().copied().unwrap_or(0);
        Ok(1)
    }
    fn stop(&mut self, _output: &mut [u8]) -> Result<usize, Nrc> {
        if !self.running {
            return Err(Nrc::RequestSequenceError);
        }
        self.running = false;
        Ok(0)
    }
    fn results(&mut self, output: &mut [u8]) -> Result<usize, Nrc> {
        output[0] = u8::from(self.running);
        Ok(1)
    }
}

struct Io(u8);
impl IoControlHandler for Io {
    fn did(&self) -> u16 {
        0x2000
    }
    fn control(&mut self, parameter: u8, input: &[u8], output: &mut [u8]) -> Result<usize, Nrc> {
        self.0 = input.first().copied().unwrap_or(parameter);
        output[0] = self.0;
        Ok(1)
    }
}

#[test]
fn routine_and_io_registries_have_application_owned_state() {
    let mut routine = Routine::default();
    let mut routines: [&mut dyn RoutineHandler; 1] = [&mut routine];
    let mut registry = RoutineRegistry::new(&mut routines);
    let mut response = [0; 16];
    assert_eq!(
        registry.process(&[0x31, 1, 0x12, 0x34, 9], &mut response),
        Ok(5)
    );
    assert_eq!(
        registry.process(&[0x31, 3, 0x12, 0x34], &mut response),
        Ok(5)
    );
    assert_eq!(response[4], 1);
    assert_eq!(
        registry.process(&[0x31, 2, 0x12, 0x34], &mut response),
        Ok(4)
    );
    let mut io = Io(0);
    let mut ios: [&mut dyn IoControlHandler; 1] = [&mut io];
    let mut registry = IoControlRegistry::new(&mut ios);
    assert_eq!(
        registry.process(&[0x2f, 0x20, 0, 3, 0xaa], &mut response),
        Ok(5)
    );
    assert_eq!(response[4], 0xaa);
}

#[test]
fn dem_services_cover_masks_groups_persistence_and_recording_disable() {
    let mut dem = DemManager::<4>::new();
    dem.report_event(0x0012_3456, true);
    dem.report_event(0x0065_4321, true);
    let mut response = [0; 32];
    let length = read_dtc_information(&[0x19, 0x01, 0x08], &dem, &mut response).unwrap();
    assert_eq!(&response[..length], &[0x59, 1, 0xff, 1, 0, 2]);
    let length = read_dtc_information(&[0x19, 0x02, 0x08], &dem, &mut response).unwrap();
    assert_eq!(length, 11);
    let mut persisted = [0; 64];
    let stored = dem.serialize(&mut persisted);
    let mut restored = DemManager::<4>::new();
    assert!(restored.deserialize(&persisted[..stored]));
    assert_eq!(restored.count(), 2);
    apply_effect(ServiceEffect::SetDtcRecording(false), &mut restored);
    restored.report_event(0x00ab_cdef, true);
    assert_eq!(restored.count(), 2);
    assert_eq!(
        clear_diagnostic_information(&[0x14, 0x12, 0x34, 0x56], &mut restored, &mut response),
        Ok(1)
    );
    assert_eq!(restored.count(), 1);
    assert_eq!(
        clear_diagnostic_information(&[0x14, 0xff, 0xff, 0xff], &mut restored, &mut response),
        Ok(1)
    );
    assert_eq!(restored.count(), 0);
}

#[derive(Default)]
struct Deferred {
    started: bool,
    cancelled: bool,
}

impl AsyncJob for Deferred {
    fn start(&mut self, request: &[u8], authenticated: bool, _response: &mut [u8]) -> AsyncResult {
        self.started = authenticated && request == [0x29, 1];
        AsyncResult::Pending
    }
    fn resume(&mut self, authenticated: bool, response: &mut [u8]) -> AsyncResult {
        if !authenticated {
            return AsyncResult::Negative(Nrc::AuthenticationRequired);
        }
        response[0] = 0x69;
        AsyncResult::Positive(1)
    }
    fn cancel(&mut self) {
        self.cancelled = true;
    }
}

#[test]
fn async_jobs_resume_authenticate_exhaust_cancel_and_invalidate_tokens() {
    let mut executor = AsyncExecutor::<1, 8>::new();
    let mut job = Deferred::default();
    let mut response = [0; 8];
    let (token, result) = executor
        .submit(&[0x29, 1], true, &mut job, &mut response)
        .unwrap();
    assert_eq!(result, AsyncResult::Pending);
    assert!(job.started);
    assert_eq!(
        executor.submit(&[], true, &mut job, &mut response),
        Err(Nrc::BusyRepeatRequest)
    );
    assert_eq!(
        executor
            .resume(token, false, &mut job, &mut response)
            .unwrap()
            .result,
        AsyncResult::Negative(Nrc::AuthenticationRequired)
    );
    assert_eq!(
        executor.resume(token, true, &mut job, &mut response),
        Err(Nrc::RequestSequenceError)
    );
    let (token, _) = executor
        .submit(&[0x29, 1], true, &mut job, &mut response)
        .unwrap();
    executor.cancel(token, &mut job).unwrap();
    assert!(job.cancelled);
}
