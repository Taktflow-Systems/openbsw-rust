use bsw_transport::async_layer::{
    AsyncTransport, ProcessedListener, ProcessingResult, SubmitError,
};
use bsw_transport::pool::{AllocationRequest, BusPolicy, MessagePool, ProviderError};
use bsw_transport::routing::{
    Endpoint, PduWriter, RouteDeclaration, RouteError, RoutingTable, TableError,
};
use bsw_transport::simple_router::{
    SimpleRoute, SimpleRouteResult, SimpleRouter, TransportEndpoint,
};
use bsw_transport::{LogicalAddress, TransportMessage, TransportResult};

fn request(size: usize, peek: &[u8]) -> AllocationRequest<'_> {
    AllocationRequest {
        source_bus: 2,
        source: 0x00f1,
        target: 0x0006,
        size,
        peek,
    }
}

#[test]
fn pool_exhaustion_release_and_generation_reuse_match_provider_contract() {
    let mut pool: MessagePool<2, 8> = MessagePool::new(BusPolicy { source_bus: 2 });
    let first = pool.acquire(request(3, &[0x22])).unwrap();
    let second = pool.acquire(request(2, &[])).unwrap();
    assert_eq!(
        pool.acquire(request(1, &[])),
        Err(ProviderError::NoMessageAvailable)
    );
    assert_eq!(pool.get(first).unwrap().payload(), &[0x22]);
    pool.get_mut(first).unwrap().append(&[0xf1, 0x90]).unwrap();
    assert_eq!(pool.get(first).unwrap().payload(), &[0x22, 0xf1, 0x90]);
    pool.release(first).unwrap();
    assert_eq!(pool.release(first), Err(ProviderError::InvalidHandle));

    let reused = pool.acquire(request(1, &[0x3e])).unwrap();
    assert_eq!(reused.slot(), first.slot());
    assert_ne!(reused.generation(), first.generation());
    assert_eq!(pool.get(first), Err(ProviderError::InvalidHandle));
    pool.release(second).unwrap();
    pool.release(reused).unwrap();
    assert_eq!(pool.in_use(), 0);
}

#[test]
fn provider_reports_responsibility_address_and_size_failures() {
    let mut pool: MessagePool<1, 4> = MessagePool::new(BusPolicy { source_bus: 2 });
    let mut wrong_bus = request(1, &[]);
    wrong_bus.source_bus = 3;
    assert_eq!(pool.acquire(wrong_bus), Err(ProviderError::NotResponsible));
    let mut invalid_source = request(1, &[]);
    invalid_source.source = u16::MAX;
    assert_eq!(
        pool.acquire(invalid_source),
        Err(ProviderError::InvalidSourceAddress)
    );
    assert_eq!(
        pool.acquire(request(5, &[])),
        Err(ProviderError::SizeTooLarge)
    );
}

#[derive(Default)]
struct Completions(Vec<(u16, u8, ProcessingResult)>);

impl ProcessedListener<8> for Completions {
    fn transport_message_processed(
        &mut self,
        token: bsw_transport::async_layer::SendToken,
        message: &TransportMessage<8>,
        result: ProcessingResult,
    ) {
        self.0.push((token.slot(), message.payload()[0], result));
    }
}

fn message(value: u8) -> TransportMessage<8> {
    let mut message = TransportMessage::new();
    message.append(&[value]).unwrap();
    message
}

#[test]
fn callbacks_are_submission_ordered_and_stale_completion_is_ignored() {
    let mut layer: AsyncTransport<2, 8> = AsyncTransport::new();
    let first = layer.submit(message(1)).unwrap();
    let second = layer.submit(message(2)).unwrap();
    assert_eq!(layer.submit(message(3)), Err(SubmitError::Busy));
    assert!(layer.complete(second, ProcessingResult::Ok));
    assert!(layer.cancel(first));
    assert!(!layer.complete(first, ProcessingResult::Ok));

    let mut callbacks = Completions::default();
    assert!(layer.dispatch_one(&mut callbacks));
    assert!(layer.dispatch_one(&mut callbacks));
    assert_eq!(
        callbacks.0,
        [
            (first.slot(), 1, ProcessingResult::Aborted),
            (second.slot(), 2, ProcessingResult::Ok)
        ]
    );
    assert!(!layer.complete(first, ProcessingResult::Ok));
}

#[test]
fn shutdown_aborts_all_jobs_before_restart() {
    let mut layer: AsyncTransport<2, 8> = AsyncTransport::new();
    layer.submit(message(1)).unwrap();
    layer.submit(message(2)).unwrap();
    layer.shutdown();
    assert_eq!(layer.submit(message(3)), Err(SubmitError::ShuttingDown));
    assert!(!layer.restart());
    let mut callbacks = Completions::default();
    while layer.dispatch_one(&mut callbacks) {}
    assert!(callbacks
        .0
        .iter()
        .all(|entry| entry.2 == ProcessingResult::Aborted));
    assert!(layer.restart());
    assert!(layer.submit(message(3)).is_ok());
}

struct Layer {
    bus: u8,
    result: TransportResult,
    received: Vec<Vec<u8>>,
}

impl TransportEndpoint<16> for Layer {
    fn bus_id(&self) -> u8 {
        self.bus
    }

    fn send(&mut self, message: &TransportMessage<16>) -> TransportResult {
        if self.result == TransportResult::Ok {
            self.received.push(message.payload().to_vec());
        }
        self.result
    }
}

#[test]
fn simple_router_supports_multi_layer_broadcast_and_backpressure() {
    let router = SimpleRouter::new([
        SimpleRoute {
            source_bus: Some(1),
            target: LogicalAddress::BROADCAST,
            destination_bus: 2,
            broadcast: true,
        },
        SimpleRoute {
            source_bus: Some(1),
            target: LogicalAddress::BROADCAST,
            destination_bus: 3,
            broadcast: true,
        },
    ]);
    let mut good = Layer {
        bus: 2,
        result: TransportResult::Ok,
        received: Vec::new(),
    };
    let mut busy = Layer {
        bus: 3,
        result: TransportResult::BufferFull,
        received: Vec::new(),
    };
    let mut layers: [&mut dyn TransportEndpoint<16>; 2] = [&mut good, &mut busy];
    let mut message = TransportMessage::new();
    message.set_target_address(0x1234);
    message.append(&[0x22, 0xf1, 0x90]).unwrap();
    assert_eq!(
        router.route(1, &message, &mut layers),
        SimpleRouteResult::Routed(1)
    );
    assert_eq!(good.received, [vec![0x22, 0xf1, 0x90]]);

    good.result = TransportResult::BufferFull;
    let mut layers: [&mut dyn TransportEndpoint<16>; 2] = [&mut good, &mut busy];
    assert_eq!(
        router.route(1, &message, &mut layers),
        SimpleRouteResult::Backpressure
    );
}

#[derive(Default)]
struct Writer {
    writes: Vec<(u32, Vec<u8>)>,
    busy: bool,
}

impl PduWriter for Writer {
    fn write(&mut self, message_id: u32, payload: &[u8]) -> Result<(), RouteError> {
        if self.busy {
            Err(RouteError::Backpressure)
        } else {
            self.writes.push((message_id, payload.to_vec()));
            Ok(())
        }
    }
}

#[test]
fn general_routes_are_declarative_and_report_errors() {
    let source = Endpoint {
        channel: 0,
        message_id: 0x100,
    };
    let table: RoutingTable<2, 3> = RoutingTable::new([
        RouteDeclaration {
            source,
            destination: Endpoint {
                channel: 1,
                message_id: 0x200,
            },
        },
        RouteDeclaration {
            source,
            destination: Endpoint {
                channel: 2,
                message_id: 0x300,
            },
        },
    ])
    .unwrap();
    let mut one = Writer::default();
    let mut two = Writer::default();
    {
        let mut writers: [Option<&mut dyn PduWriter>; 3] = [None, Some(&mut one), Some(&mut two)];
        assert_eq!(table.route(source, &[1, 2, 3], &mut writers), Ok(2));
    }
    assert_eq!(one.writes, [(0x200, vec![1, 2, 3])]);
    assert_eq!(two.writes, [(0x300, vec![1, 2, 3])]);
    let mut no_writers: [Option<&mut dyn PduWriter>; 3] = [None, None, None];
    assert_eq!(
        table.route(
            Endpoint {
                channel: 0,
                message_id: 9
            },
            &[],
            &mut no_writers
        ),
        Err(RouteError::NoRoute)
    );
}

#[test]
fn routing_table_rejects_invalid_declarations() {
    let endpoint = Endpoint {
        channel: 0,
        message_id: 1,
    };
    assert!(matches!(
        RoutingTable::<1, 1>::new([RouteDeclaration {
            source: endpoint,
            destination: endpoint
        }]),
        Err(TableError::SelfRoute)
    ));
    assert!(matches!(
        RoutingTable::<1, 1>::new([RouteDeclaration {
            source: endpoint,
            destination: Endpoint {
                channel: 1,
                message_id: 2
            },
        }]),
        Err(TableError::ChannelOutOfRange)
    ));
}
