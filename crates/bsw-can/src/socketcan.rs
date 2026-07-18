//! Linux SocketCAN backend (package D15).
//!
//! [`SocketCan`] wraps a nonblocking `PF_CAN`/`SOCK_RAW` socket: classic
//! and CAN FD frames, kernel-side acceptance filters, loopback and
//! receive-own-messages control, error-frame reception with decoded error
//! classes, and kernel receive timestamps (`SO_TIMESTAMP`).
//!
//! All FFI is confined to this module; the rest of the crate sees only
//! [`CanFrame`], [`crate::filter`] types, and typed errors. Integration
//! tests run against a `vcan` interface on Linux CI.

use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};

use crate::can_id::CanId;
use crate::frame::CanFrame;
use crate::transceiver::{CanTransceiver, ErrorCode, State, TransceiverState};

/// Maximum number of kernel acceptance filters accepted by
/// [`SocketCan::set_filters`].
pub const MAX_KERNEL_FILTERS: usize = 64;

/// One kernel acceptance filter: a frame matches when
/// `received_id & mask == id & mask`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KernelFilter {
    /// Comparison identifier (with `CAN_EFF_FLAG` for extended IDs).
    pub id: u32,
    /// Comparison mask.
    pub mask: u32,
}

impl KernelFilter {
    /// Match exactly one base or extended identifier.
    #[must_use]
    pub const fn exact(id: CanId) -> Self {
        let raw = if id.is_extended() {
            id.value() | libc::CAN_EFF_FLAG
        } else {
            id.value()
        };
        let mask = libc::CAN_EFF_FLAG | libc::CAN_RTR_FLAG | libc::CAN_EFF_MASK;
        Self { id: raw, mask }
    }
}

/// Decoded CAN error frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools)] // mirrors the kernel's error-class bit set
pub struct ErrorFrame {
    /// Raw error class bits from the frame identifier.
    pub class: u32,
    /// The controller signalled bus off.
    pub bus_off: bool,
    /// Controller error-state details (warning/passive) present.
    pub controller_problem: bool,
    /// Protocol violation reported.
    pub protocol_violation: bool,
    /// Transceiver (physical layer) problem reported.
    pub transceiver_problem: bool,
    /// Suggested hardware state derived from the error class.
    pub state_hint: Option<TransceiverState>,
}

impl ErrorFrame {
    fn decode(raw_id: u32, payload: &[u8]) -> Self {
        let class = raw_id & libc::CAN_EFF_MASK;
        let bus_off = class & libc::CAN_ERR_BUSOFF != 0;
        let controller_problem = class & libc::CAN_ERR_CRTL != 0;
        let protocol_violation = class & libc::CAN_ERR_PROT != 0;
        let transceiver_problem = class & libc::CAN_ERR_TRX != 0;
        let mut state_hint = None;
        if bus_off {
            state_hint = Some(TransceiverState::BusOff);
        } else if controller_problem {
            let detail = payload.get(1).copied().unwrap_or(0);
            #[allow(clippy::cast_sign_loss)]
            let passive = u32::from(detail)
                & ((libc::CAN_ERR_CRTL_RX_PASSIVE | libc::CAN_ERR_CRTL_TX_PASSIVE) as u32);
            #[allow(clippy::cast_sign_loss)]
            let active = u32::from(detail) & (libc::CAN_ERR_CRTL_ACTIVE as u32);
            if passive != 0 {
                state_hint = Some(TransceiverState::Passive);
            } else if active != 0 {
                state_hint = Some(TransceiverState::Active);
            }
        }
        Self {
            class,
            bus_off,
            controller_problem,
            protocol_violation,
            transceiver_problem,
            state_hint,
        }
    }
}

/// One received event.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RxEvent {
    /// A data frame; its timestamp is the kernel receive time in
    /// microseconds (wrapping).
    Frame(CanFrame),
    /// An error frame.
    Error(ErrorFrame),
}

/// Nonblocking SocketCAN endpoint.
#[derive(Debug)]
pub struct SocketCan {
    fd: OwnedFd,
    fd_frames: bool,
    logical_state: State,
    error_state: TransceiverState,
}

impl SocketCan {
    /// Open a nonblocking raw CAN socket bound to `interface` (classic
    /// frames only).
    pub fn open(interface: &str) -> io::Result<Self> {
        Self::open_inner(interface, false)
    }

    /// Open with CAN FD frame support enabled.
    pub fn open_fd(interface: &str) -> io::Result<Self> {
        Self::open_inner(interface, true)
    }

    fn open_inner(interface: &str, fd_frames: bool) -> io::Result<Self> {
        // SAFETY: plain socket(2) call; the result is checked before use.
        let raw = unsafe {
            libc::socket(
                libc::PF_CAN,
                libc::SOCK_RAW | libc::SOCK_NONBLOCK | libc::SOCK_CLOEXEC,
                libc::CAN_RAW,
            )
        };
        if raw < 0 {
            return Err(io::Error::last_os_error());
        }
        // SAFETY: `raw` is a freshly created, owned, valid descriptor.
        let fd = unsafe { OwnedFd::from_raw_fd(raw) };

        let mut name = [0u8; libc::IFNAMSIZ];
        let bytes = interface.as_bytes();
        if bytes.is_empty() || bytes.len() >= libc::IFNAMSIZ {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "invalid CAN interface name",
            ));
        }
        name[..bytes.len()].copy_from_slice(bytes);
        // SAFETY: `name` is a NUL-terminated buffer of IFNAMSIZ bytes.
        let index = unsafe { libc::if_nametoindex(name.as_ptr().cast()) };
        if index == 0 {
            return Err(io::Error::last_os_error());
        }

        // SAFETY: zero-initialised sockaddr_can is a valid representation.
        let mut address: libc::sockaddr_can = unsafe { std::mem::zeroed() };
        address.can_family = libc::AF_CAN as libc::sa_family_t;
        #[allow(clippy::cast_possible_wrap)]
        {
            address.can_ifindex = index as libc::c_int;
        }
        // SAFETY: `address` outlives the call and its size is passed.
        let bound = unsafe {
            libc::bind(
                fd.as_raw_fd(),
                std::ptr::addr_of!(address).cast(),
                std::mem::size_of::<libc::sockaddr_can>() as libc::socklen_t,
            )
        };
        if bound < 0 {
            return Err(io::Error::last_os_error());
        }

        let socket = Self {
            fd,
            fd_frames,
            logical_state: State::Initialized,
            error_state: TransceiverState::Active,
        };
        if fd_frames {
            socket.set_option(libc::SOL_CAN_RAW, libc::CAN_RAW_FD_FRAMES, 1)?;
        }
        // Kernel receive timestamps for every frame.
        socket.set_option(libc::SOL_SOCKET, libc::SO_TIMESTAMP, 1)?;
        Ok(socket)
    }

    fn set_option(
        &self,
        level: libc::c_int,
        option: libc::c_int,
        value: libc::c_int,
    ) -> io::Result<()> {
        // SAFETY: `value` outlives the call; size is passed alongside.
        let result = unsafe {
            libc::setsockopt(
                self.fd.as_raw_fd(),
                level,
                option,
                std::ptr::addr_of!(value).cast(),
                std::mem::size_of::<libc::c_int>() as libc::socklen_t,
            )
        };
        if result < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(())
        }
    }

    /// Raw descriptor for poll/select integration.
    #[must_use]
    pub fn raw_fd(&self) -> RawFd {
        self.fd.as_raw_fd()
    }

    /// Install kernel acceptance filters (up to [`MAX_KERNEL_FILTERS`]).
    ///
    /// An empty list rejects every frame; use [`SocketCan::open_filter`]
    /// to accept everything again.
    pub fn set_filters(&self, filters: &[KernelFilter]) -> io::Result<()> {
        if filters.len() > MAX_KERNEL_FILTERS {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "too many kernel filters",
            ));
        }
        let mut raw: [libc::can_filter; MAX_KERNEL_FILTERS] =
            // SAFETY: can_filter is two plain u32 fields; zeroed is valid.
            unsafe { std::mem::zeroed() };
        for (slot, filter) in raw.iter_mut().zip(filters.iter()) {
            slot.can_id = filter.id;
            slot.can_mask = filter.mask;
        }
        let bytes = std::mem::size_of::<libc::can_filter>() * filters.len();
        // SAFETY: `raw` holds at least `bytes` valid bytes for the call.
        let result = unsafe {
            libc::setsockopt(
                self.fd.as_raw_fd(),
                libc::SOL_CAN_RAW,
                libc::CAN_RAW_FILTER,
                raw.as_ptr().cast(),
                bytes as libc::socklen_t,
            )
        };
        if result < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(())
        }
    }

    /// Accept every data frame (the kernel default).
    pub fn open_filter(&self) -> io::Result<()> {
        self.set_filters(&[KernelFilter { id: 0, mask: 0 }])
    }

    /// Enable or disable kernel loopback of frames between local sockets.
    pub fn set_loopback(&self, enabled: bool) -> io::Result<()> {
        self.set_option(
            libc::SOL_CAN_RAW,
            libc::CAN_RAW_LOOPBACK,
            libc::c_int::from(enabled),
        )
    }

    /// Deliver this socket's own transmissions back to it.
    pub fn set_receive_own(&self, enabled: bool) -> io::Result<()> {
        self.set_option(
            libc::SOL_CAN_RAW,
            libc::CAN_RAW_RECV_OWN_MSGS,
            libc::c_int::from(enabled),
        )
    }

    /// Subscribe to every error frame class.
    pub fn enable_error_frames(&self) -> io::Result<()> {
        #[allow(clippy::cast_possible_wrap)]
        self.set_option(
            libc::SOL_CAN_RAW,
            libc::CAN_RAW_ERR_FILTER,
            libc::CAN_ERR_MASK as libc::c_int,
        )
    }

    /// Send one frame. `WouldBlock` maps to a full transmit queue.
    pub fn send(&self, frame: &CanFrame) -> io::Result<()> {
        let raw_id = if frame.id().is_extended() {
            frame.id().value() | libc::CAN_EFF_FLAG
        } else {
            frame.id().value()
        };
        let payload = frame.payload();
        let written = if self.fd_frames && payload.len() > 8 {
            // SAFETY: zeroed canfd_frame is a valid representation.
            let mut fd_frame: libc::canfd_frame = unsafe { std::mem::zeroed() };
            fd_frame.can_id = raw_id;
            let padded = crate::dlc::padded_length(payload.len(), true).ok_or_else(|| {
                io::Error::new(io::ErrorKind::InvalidInput, "payload too long for CAN FD")
            })?;
            #[allow(clippy::cast_possible_truncation)]
            {
                fd_frame.len = padded as u8;
            }
            fd_frame.data[..payload.len()].copy_from_slice(payload);
            // SAFETY: `fd_frame` outlives the call; its exact size is passed.
            unsafe {
                libc::write(
                    self.fd.as_raw_fd(),
                    std::ptr::addr_of!(fd_frame).cast(),
                    std::mem::size_of::<libc::canfd_frame>(),
                )
            }
        } else {
            if payload.len() > 8 {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "payload above 8 bytes needs an FD socket",
                ));
            }
            // SAFETY: zeroed can_frame is a valid representation.
            let mut classic: libc::can_frame = unsafe { std::mem::zeroed() };
            classic.can_id = raw_id;
            #[allow(clippy::cast_possible_truncation)]
            {
                classic.can_dlc = payload.len() as u8;
            }
            classic.data[..payload.len()].copy_from_slice(payload);
            // SAFETY: `classic` outlives the call; its exact size is passed.
            unsafe {
                libc::write(
                    self.fd.as_raw_fd(),
                    std::ptr::addr_of!(classic).cast(),
                    std::mem::size_of::<libc::can_frame>(),
                )
            }
        };
        if written < 0 {
            Err(io::Error::last_os_error())
        } else {
            Ok(())
        }
    }

    /// Receive the next pending event without blocking.
    ///
    /// Returns `Ok(None)` when no frame is queued.
    pub fn receive(&mut self) -> io::Result<Option<RxEvent>> {
        let mut buffer = [0u8; std::mem::size_of::<libc::canfd_frame>()];
        let mut control = [0u8; 64];
        let mut iov = libc::iovec {
            iov_base: buffer.as_mut_ptr().cast(),
            iov_len: buffer.len(),
        };
        // SAFETY: zeroed msghdr is valid; pointers set right below.
        let mut msg: libc::msghdr = unsafe { std::mem::zeroed() };
        msg.msg_iov = std::ptr::addr_of_mut!(iov);
        msg.msg_iovlen = 1;
        msg.msg_control = control.as_mut_ptr().cast();
        msg.msg_controllen = control.len();

        // SAFETY: `msg` and the buffers it references outlive the call.
        let received = unsafe { libc::recvmsg(self.fd.as_raw_fd(), &mut msg, 0) };
        if received < 0 {
            let error = io::Error::last_os_error();
            if error.kind() == io::ErrorKind::WouldBlock {
                return Ok(None);
            }
            return Err(error);
        }
        let received = received as usize;
        let timestamp_us = Self::timestamp_from_control(&msg);

        // Interpret the datagram as classic or FD frame by size.
        let (raw_id, payload_len, data_offset) =
            if received >= std::mem::size_of::<libc::canfd_frame>() {
                let raw_id = u32::from_ne_bytes(buffer[0..4].try_into().expect("4 bytes"));
                let len = usize::from(buffer[4]);
                (raw_id, len.min(64), 8)
            } else {
                let raw_id = u32::from_ne_bytes(buffer[0..4].try_into().expect("4 bytes"));
                let len = usize::from(buffer[4]).min(8);
                (raw_id, len, 8)
            };

        if raw_id & libc::CAN_ERR_FLAG != 0 {
            let payload = &buffer[data_offset..data_offset + payload_len];
            let event = ErrorFrame::decode(raw_id, payload);
            if let Some(state) = event.state_hint {
                self.error_state = state;
            }
            return Ok(Some(RxEvent::Error(event)));
        }

        let extended = raw_id & libc::CAN_EFF_FLAG != 0;
        let value = if extended {
            raw_id & libc::CAN_EFF_MASK
        } else {
            raw_id & libc::CAN_SFF_MASK
        };
        let payload_len = payload_len.min(crate::frame::MAX_FRAME_LENGTH);
        let payload = &buffer[data_offset..data_offset + payload_len];
        let mut frame = CanFrame::with_raw_id(value, payload, extended);
        frame.set_timestamp(timestamp_us);
        Ok(Some(RxEvent::Frame(frame)))
    }

    /// Extract the `SO_TIMESTAMP` microsecond stamp from ancillary data.
    fn timestamp_from_control(msg: &libc::msghdr) -> u32 {
        // SAFETY: CMSG_FIRSTHDR only reads header fields of `msg`.
        let mut header = unsafe { libc::CMSG_FIRSTHDR(msg) };
        while !header.is_null() {
            // SAFETY: `header` came from CMSG_FIRSTHDR/CMSG_NXTHDR over a
            // valid msghdr, so reading the header struct is in bounds.
            let (level, kind) = unsafe { ((*header).cmsg_level, (*header).cmsg_type) };
            if level == libc::SOL_SOCKET && kind == libc::SCM_TIMESTAMP {
                // SAFETY: CMSG_DATA points at a timeval payload for
                // SCM_TIMESTAMP; read it unaligned to be safe.
                let time = unsafe {
                    libc::CMSG_DATA(header)
                        .cast::<libc::timeval>()
                        .read_unaligned()
                };
                let micros = i128::from(time.tv_sec) * 1_000_000 + i128::from(time.tv_usec);
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                return micros as u32;
            }
            // SAFETY: header chain iteration over the same valid msghdr.
            header = unsafe { libc::CMSG_NXTHDR(msg, header) };
        }
        0
    }
}

impl CanTransceiver for SocketCan {
    fn init(&mut self) -> ErrorCode {
        self.logical_state = State::Initialized;
        ErrorCode::Ok
    }

    fn shutdown(&mut self) {
        self.logical_state = State::Closed;
    }

    fn open(&mut self) -> ErrorCode {
        if self.logical_state != State::Initialized && self.logical_state != State::Muted {
            return ErrorCode::IllegalState;
        }
        self.logical_state = State::Open;
        ErrorCode::Ok
    }

    fn open_with_frame(&mut self, frame: &CanFrame) -> ErrorCode {
        let code = CanTransceiver::open(self);
        if code != ErrorCode::Ok {
            return code;
        }
        CanTransceiver::write(self, frame)
    }

    fn close(&mut self) -> ErrorCode {
        if self.logical_state == State::Closed {
            return ErrorCode::IllegalState;
        }
        self.logical_state = State::Initialized;
        ErrorCode::Ok
    }

    fn mute(&mut self) -> ErrorCode {
        if self.logical_state != State::Open {
            return ErrorCode::IllegalState;
        }
        self.logical_state = State::Muted;
        ErrorCode::Ok
    }

    fn unmute(&mut self) -> ErrorCode {
        if self.logical_state != State::Muted {
            return ErrorCode::IllegalState;
        }
        self.logical_state = State::Open;
        ErrorCode::Ok
    }

    fn state(&self) -> State {
        self.logical_state
    }

    fn baudrate(&self) -> u32 {
        crate::transceiver::BAUDRATE_HIGHSPEED
    }

    fn hw_queue_timeout(&self) -> u16 {
        0
    }

    fn bus_id(&self) -> u8 {
        0
    }

    fn write(&mut self, frame: &CanFrame) -> ErrorCode {
        if self.logical_state == State::Muted {
            return ErrorCode::TxOffline;
        }
        if self.logical_state != State::Open {
            return ErrorCode::TxOffline;
        }
        match self.send(frame) {
            Ok(()) => ErrorCode::Ok,
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => ErrorCode::TxHwQueueFull,
            Err(_) => ErrorCode::TxFail,
        }
    }

    fn transceiver_state(&self) -> TransceiverState {
        self.error_state
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_frame_decoding_maps_classes_and_state_hints() {
        let bus_off = ErrorFrame::decode(libc::CAN_ERR_FLAG | libc::CAN_ERR_BUSOFF, &[0; 8]);
        assert!(bus_off.bus_off);
        assert_eq!(bus_off.state_hint, Some(TransceiverState::BusOff));

        let mut payload = [0u8; 8];
        #[allow(clippy::cast_possible_truncation)]
        {
            payload[1] = libc::CAN_ERR_CRTL_TX_PASSIVE as u8;
        }
        let passive = ErrorFrame::decode(libc::CAN_ERR_FLAG | libc::CAN_ERR_CRTL, &payload);
        assert!(passive.controller_problem);
        assert_eq!(passive.state_hint, Some(TransceiverState::Passive));

        let protocol = ErrorFrame::decode(libc::CAN_ERR_FLAG | libc::CAN_ERR_PROT, &[0; 8]);
        assert!(protocol.protocol_violation);
        assert_eq!(protocol.state_hint, None);
    }

    #[test]
    fn kernel_filter_exact_matches_expected_masks() {
        let base = KernelFilter::exact(CanId::base(0x123));
        assert_eq!(base.id, 0x123);
        let extended = KernelFilter::exact(CanId::extended(0x1234_5678));
        assert_eq!(extended.id, 0x1234_5678 | libc::CAN_EFF_FLAG);
    }

    #[test]
    fn invalid_interface_names_are_rejected() {
        assert!(SocketCan::open("").is_err());
        assert!(SocketCan::open("this-interface-name-is-way-too-long").is_err());
    }
}
