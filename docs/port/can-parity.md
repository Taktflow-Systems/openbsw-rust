# CAN (cpp2can) parity map — package D12

Pinned upstream: `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`, module
`libs/bsw/cpp2can`. Every upstream public API surface is assigned below;
"native replacement" rows keep upstream-observable behavior with a Rust-
idiomatic mechanism and cite where the equivalence is proven.

| Upstream API (include/can/...) | Assignment | Rust location | Notes / evidence |
|---|---|---|---|
| `canframes/CANFrame.h` | port | `bsw-can::frame::CanFrame` | payload, id, timestamp; equality ignores timestamps. Tests: `crates/bsw-can/tests/can_transceiver.rs`. |
| `canframes/CanId.h` | port | `bsw-can::can_id::CanId` | base/extended/force-no-FD encoding. |
| `filter/IFilter.h` | port | `bsw-can::filter::Filter` trait | add/add_range/matches/clear/open/merge. |
| `filter/BitFieldFilter.h` | port | `bsw-can::filter::BitFieldFilter` | 2048-bit mask, merge semantics. |
| `filter/IntervalFilter.h` | port | `bsw-can::filter::IntervalFilter` | inclusive bounds. |
| `filter/AbstractStaticBitFieldFilter.h` | native replacement | `BitFieldFilter` + const tables | static mask expressed as const-initialized filter; no separate class needed. |
| `framemgmt/ICANFrameListener.h` | native replacement | `bsw-can::listener::FrameListener` | intrusive `etl` list becomes fixed-capacity `FrameDispatcher` with `&mut` registrations; per-listener filter becomes `matches()`. Tests: `listener.rs` (order, capacity, reentrancy). |
| `framemgmt/IFilteredCANFrameSentListener.h` | native replacement | `bsw-can::listener::FrameListener` | same shape as RX path; registered on a TX-confirmation dispatcher instance. |
| `framemgmt/ICANFrameSentListener.h` | native replacement | `bsw-can::listener::FrameListener` | unfiltered variant = default `matches` (always true). |
| `framemgmt/AbstractBitFieldFilteredCANFrameListener.h` | native replacement | listener impl owning a `BitFieldFilter` | listener composes a filter value; no inheritance needed. |
| `framemgmt/AbstractIntervalFilteredCANFrameListener.h` | native replacement | listener impl owning an `IntervalFilter` | see `listener.rs` test `dispatch_delivers_in_registration_order_with_filters`. |
| `framemgmt/IMerger.h` | native replacement | `Filter::merge_into` | merger interface folded into the filter trait. |
| `transceiver/ICanTransceiver.h` | port | `bsw-can::transceiver::CanTransceiver` | init/shutdown/open/close/mute/unmute/write/states/baudrate/bus id. |
| `transceiver/AbstractCANTransceiver.h` | port | `bsw-can::transceiver::AbstractTransceiver` | state machine, filter, statistics, listener slots. |
| `transceiver/ICANTransceiverStateListener.h` | port | `bsw-can::listener::StateListener` | `state_changed` + `phy_error`; states map to `TransceiverState`. |
| `transceiver/statistics.h` | port | `bsw-can::transceiver::Statistics` + `bsw-can::error_state::BusLoadEstimator` | counters plus windowed bus-load (D14). |
| `CanLogger.h` | native replacement | `bsw-logger` component id | logging is generic; a CAN component id replaces the dedicated logger header. |
| — (implicit ISO 11898-1 fault confinement) | port | `bsw-can::error_state::ErrorStateTracker` | TEC/REC thresholds 128/256, bus-off recovery (D14 tests). |
| — (CAN FD DLC mapping) | port | `bsw-can::dlc` | DLC 9..=15 stepped lengths, padding (D14 tests). |
| — (host virtual bus for tests) | project extension | `bsw-can::virtual_bus` | deterministic in-process bus with fault injection (D16). |
| — (Linux SocketCAN adapter) | project extension | `bsw-can::socketcan` | kernel filters, loopback, error frames, timestamps (D15); upstream's POSIX platform uses its own adapter, ours targets `PF_CAN` directly. |

## Open items deferred to later packages

- Hardware driver convergence to the common receive contract (BSP
  `CanReceiver` split) is G03/G04 scope.
- DoCAN-level connection semantics are E09-E13 scope.
