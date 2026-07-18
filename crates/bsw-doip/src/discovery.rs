//! UDP vehicle discovery, announcements, and entity status (E27).

use bsw_time::{Duration, Instant};

use crate::{
    DiagnosticPowerMode, EntityStatus, Packet, Payload, ProtocolVersion, VehicleAnnouncement,
    VehicleIdentification,
};

/// Deterministic unsolicited-announcement schedule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AnnouncementSchedule {
    /// Delay from network availability to the first announcement.
    pub initial_delay: Duration,
    /// Interval between announcements.
    pub interval: Duration,
    /// Number of announcement datagrams in one burst.
    pub count: u8,
}

/// Allocation-free DoIP discovery entity.
pub struct DiscoveryEntity {
    announcement: VehicleAnnouncement,
    status: EntityStatus,
    power_mode: DiagnosticPowerMode,
    schedule: AnnouncementSchedule,
    next_announcement: Instant,
    announcements_remaining: u8,
}

impl DiscoveryEntity {
    /// Create an idle entity. Call [`start_announcements`](Self::start_announcements)
    /// whenever a usable network address becomes available.
    pub const fn new(
        announcement: VehicleAnnouncement,
        status: EntityStatus,
        power_mode: DiagnosticPowerMode,
        schedule: AnnouncementSchedule,
    ) -> Self {
        Self {
            announcement,
            status,
            power_mode,
            schedule,
            next_announcement: Instant::from_nanos(0),
            announcements_remaining: 0,
        }
    }

    /// Start or restart the configured announcement burst.
    pub fn start_announcements(&mut self, now: Instant) {
        self.next_announcement = now.wrapping_add(self.schedule.initial_delay);
        self.announcements_remaining = self.schedule.count;
    }

    /// Stop an in-flight announcement burst.
    pub fn stop_announcements(&mut self) {
        self.announcements_remaining = 0;
    }

    /// Update live entity socket occupancy.
    pub fn set_open_sockets(&mut self, open: u8) -> bool {
        if open > self.status.max_sockets {
            return false;
        }
        self.status.open_sockets = open;
        true
    }

    /// Update the diagnostic power mode returned to clients.
    pub fn set_power_mode(&mut self, mode: DiagnosticPowerMode) {
        self.power_mode = mode;
    }

    /// Decode a UDP request and encode the corresponding unicast response.
    /// `Ok(0)` means the request was valid but did not target this entity.
    pub fn handle_datagram(
        &self,
        input: &[u8],
        output: &mut [u8],
    ) -> Result<usize, crate::CodecError> {
        let packet = Packet::parse(input)?;
        let response = match packet.payload {
            Payload::VehicleIdentification(selector) if self.matches(selector) => {
                Some(Payload::VehicleAnnouncement(self.announcement.clone()))
            }
            Payload::VehicleIdentification(_) => None,
            Payload::EntityStatusRequest => Some(Payload::EntityStatusResponse(self.status)),
            Payload::PowerModeRequest => Some(Payload::PowerModeResponse(self.power_mode)),
            _ => None,
        };
        match response {
            Some(payload) => Packet {
                version: packet.version,
                payload,
            }
            .encode(output),
            None => Ok(0),
        }
    }

    /// Encode one due unsolicited announcement. Exact deadlines are accepted.
    pub fn poll_announcement(
        &mut self,
        now: Instant,
        output: &mut [u8],
    ) -> Result<usize, crate::CodecError> {
        if self.announcements_remaining == 0 || !now.is_at_or_after(self.next_announcement) {
            return Ok(0);
        }
        self.announcements_remaining -= 1;
        self.next_announcement = self.next_announcement.wrapping_add(self.schedule.interval);
        Packet {
            version: ProtocolVersion::Iso2012,
            payload: Payload::VehicleAnnouncement(self.announcement.clone()),
        }
        .encode(output)
    }

    /// Remaining frames in the current announcement burst.
    pub const fn announcements_remaining(&self) -> u8 {
        self.announcements_remaining
    }

    fn matches(&self, selector: VehicleIdentification) -> bool {
        match selector {
            VehicleIdentification::All => true,
            VehicleIdentification::Eid(eid) => eid == self.announcement.eid,
            VehicleIdentification::Vin(vin) => vin == self.announcement.vin,
        }
    }
}

/// POSIX service loop built on the `bsw-ethernet` UDP adapter.
#[cfg(feature = "std")]
pub mod posix {
    use bsw_ethernet::{
        endpoint::IpEndpoint,
        ip::IpAddress,
        posix::udp::PosixUdpSocket,
        udp::{DatagramPacket, UdpError, UdpSocket},
    };
    use bsw_time::Instant;

    use super::DiscoveryEntity;

    /// Non-blocking DoIP discovery service.
    pub struct PosixDiscoveryService {
        socket: PosixUdpSocket,
        entity: DiscoveryEntity,
    }

    impl PosixDiscoveryService {
        /// Bind one service endpoint, including ephemeral port 0 for tests.
        pub fn bind(
            address: IpAddress,
            port: u16,
            entity: DiscoveryEntity,
        ) -> Result<Self, UdpError> {
            let mut socket = PosixUdpSocket::new();
            if socket.bind(Some(&address), port) != UdpError::Ok {
                return Err(UdpError::NotOk);
            }
            Ok(Self { socket, entity })
        }

        /// Bound local port.
        pub fn local_port(&self) -> u16 {
            self.socket.local_port()
        }

        /// Access dynamic entity configuration.
        pub fn entity_mut(&mut self) -> &mut DiscoveryEntity {
            &mut self.entity
        }

        /// Stop the socket by consuming the service and return its entity
        /// state so a lifecycle owner can bind it again during restart.
        pub fn into_entity(self) -> DiscoveryEntity {
            self.entity
        }

        /// Process at most one client datagram without blocking.
        pub fn poll_client(
            &mut self,
            input: &mut [u8],
            output: &mut [u8],
        ) -> Result<usize, UdpError> {
            let Some((length, source)) = self.socket.poll_recv_from(input)? else {
                return Ok(0);
            };
            let response_length = self
                .entity
                .handle_datagram(&input[..length], output)
                .map_err(|_| UdpError::NotOk)?;
            if response_length == 0 {
                return Ok(0);
            }
            self.send_to(source, &output[..response_length])?;
            Ok(response_length)
        }

        /// Send one due announcement to a caller-selected broadcast/multicast target.
        pub fn poll_announcement(
            &mut self,
            now: Instant,
            target: IpEndpoint,
            output: &mut [u8],
        ) -> Result<usize, UdpError> {
            let length = self
                .entity
                .poll_announcement(now, output)
                .map_err(|_| UdpError::NotOk)?;
            if length == 0 {
                return Ok(0);
            }
            self.send_to(target, &output[..length])?;
            Ok(length)
        }

        fn send_to(&mut self, endpoint: IpEndpoint, bytes: &[u8]) -> Result<(), UdpError> {
            let packet = DatagramPacket::with_endpoint(bytes, endpoint);
            match self.socket.send_to(&packet) {
                UdpError::Ok => Ok(()),
                error => Err(error),
            }
        }
    }
}
