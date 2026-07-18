//! Source/target transport router with broadcast and backpressure (E03).

use crate::{LogicalAddress, TransportMessage, TransportResult};

/// Destination transport-layer boundary used by [`SimpleRouter`].
pub trait TransportEndpoint<const PAYLOAD: usize> {
    /// Stable bus identifier.
    fn bus_id(&self) -> u8;

    /// Send one routed message. The endpoint must not retain the borrow.
    fn send(&mut self, message: &TransportMessage<PAYLOAD>) -> TransportResult;
}

/// One declarative transport route.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SimpleRoute {
    /// Source bus, or `None` for any bus.
    pub source_bus: Option<u8>,
    /// Target address, or [`LogicalAddress::BROADCAST`] for any target.
    pub target: LogicalAddress,
    /// Destination bus.
    pub destination_bus: u8,
    /// Whether this route participates in broadcast fan-out.
    pub broadcast: bool,
}

/// Outcome of one routing attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimpleRouteResult {
    /// Message was accepted by this many layers.
    Routed(usize),
    /// At least one route matched but every destination applied backpressure.
    Backpressure,
    /// No route declaration matched.
    NoRoute,
}

/// Fixed declarative simple transport router.
pub struct SimpleRouter<const ROUTES: usize> {
    routes: [SimpleRoute; ROUTES],
}

impl<const ROUTES: usize> SimpleRouter<ROUTES> {
    /// Create a router from declarations evaluated in array order.
    pub const fn new(routes: [SimpleRoute; ROUTES]) -> Self {
        Self { routes }
    }

    /// Route one message across caller-owned transport layers.
    pub fn route<const PAYLOAD: usize>(
        &self,
        source_bus: u8,
        message: &TransportMessage<PAYLOAD>,
        layers: &mut [&mut dyn TransportEndpoint<PAYLOAD>],
    ) -> SimpleRouteResult {
        let target = LogicalAddress::new(message.target_address());
        let mut matched = false;
        let mut accepted = 0usize;
        for route in &self.routes {
            if route.source_bus.is_some_and(|bus| bus != source_bus)
                || (route.target != LogicalAddress::BROADCAST && route.target != target)
            {
                continue;
            }
            matched = true;
            let Some(layer) = layers
                .iter_mut()
                .find(|layer| layer.bus_id() == route.destination_bus)
            else {
                continue;
            };
            if layer.send(message) == TransportResult::Ok {
                accepted += 1;
            }
            if !route.broadcast {
                break;
            }
        }
        if accepted > 0 {
            SimpleRouteResult::Routed(accepted)
        } else if matched {
            SimpleRouteResult::Backpressure
        } else {
            SimpleRouteResult::NoRoute
        }
    }
}
