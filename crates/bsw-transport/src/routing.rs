//! General PDU routing declarations, endpoints, status, and errors (E04).

/// Route endpoint address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Endpoint {
    /// Channel index.
    pub channel: u8,
    /// Channel-specific message/PDU identifier.
    pub message_id: u32,
}

/// One source-to-destination route declaration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RouteDeclaration {
    /// Input endpoint.
    pub source: Endpoint,
    /// Output endpoint.
    pub destination: Endpoint,
}

/// Routing-table validation error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TableError {
    /// A declaration uses a channel outside the configured range.
    ChannelOutOfRange,
    /// Source and destination are identical and would loop immediately.
    SelfRoute,
    /// The table contains an exact duplicate declaration.
    Duplicate,
}

/// Runtime route status/error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouteError {
    /// No declaration matches the source endpoint.
    NoRoute,
    /// Destination channel has no writer.
    MissingEndpoint,
    /// At least one destination could not currently accept data.
    Backpressure,
}

/// Output channel interface.
pub trait PduWriter {
    /// Write a PDU using the destination-channel message identifier.
    fn write(&mut self, message_id: u32, payload: &[u8]) -> Result<(), RouteError>;
}

/// Fixed validated routing table.
pub struct RoutingTable<const ROUTES: usize, const CHANNELS: usize> {
    routes: [RouteDeclaration; ROUTES],
}

impl<const ROUTES: usize, const CHANNELS: usize> RoutingTable<ROUTES, CHANNELS> {
    /// Validate and construct a routing table.
    pub fn new(routes: [RouteDeclaration; ROUTES]) -> Result<Self, TableError> {
        for (index, route) in routes.iter().enumerate() {
            if usize::from(route.source.channel) >= CHANNELS
                || usize::from(route.destination.channel) >= CHANNELS
            {
                return Err(TableError::ChannelOutOfRange);
            }
            if route.source == route.destination {
                return Err(TableError::SelfRoute);
            }
            if routes[..index].contains(route) {
                return Err(TableError::Duplicate);
            }
        }
        Ok(Self { routes })
    }

    /// Route to every matching destination in declaration order.
    pub fn route(
        &self,
        source: Endpoint,
        payload: &[u8],
        writers: &mut [Option<&mut dyn PduWriter>; CHANNELS],
    ) -> Result<usize, RouteError> {
        let mut matched = 0usize;
        let mut sent = 0usize;
        let mut backpressure = false;
        for route in self.routes.iter().filter(|route| route.source == source) {
            matched += 1;
            let Some(writer) = writers[usize::from(route.destination.channel)].as_deref_mut()
            else {
                backpressure = true;
                continue;
            };
            match writer.write(route.destination.message_id, payload) {
                Ok(()) => sent += 1,
                Err(RouteError::Backpressure) => backpressure = true,
                Err(error) => return Err(error),
            }
        }
        if matched == 0 {
            Err(RouteError::NoRoute)
        } else if backpressure {
            Err(RouteError::Backpressure)
        } else {
            Ok(sent)
        }
    }
}
