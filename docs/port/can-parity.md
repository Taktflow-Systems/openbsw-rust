# CAN (cpp2can) parity map — package D12

Pinned upstream: `be0029bbb79fe901048a24c2665f2ba854328734` (re-pinned
2026-07-20 from `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`; see
`docs/port/repin-2026-07-20.md`), module `libs/bsw/cpp2can`. Every upstream
public API surface is assigned below;
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

## Re-pin 2026-07-20: STM32 CAN register-level gaps 33-37 (all deferred with basis)

The oracle re-pin to `be0029b` required a disposition for the five
register-level gaps recorded in
`docs/port/stm32-can-drift-comparison-2026-07-19.md` (gaps 33-37, upstream
commits `1a11d135`/`be0029bb`). All five are DEFERRED. Shared basis: each
adoption requires register-level changes to the production STM32 drivers
that are not host-testable without introducing new production seams, and
any such driver change requires target/HIL re-verification on both
reference boards; no HIL bench is available in this tranche. These five
deferrals are exactly the open items that keep the STM32 CAN cluster at
the software boundary: everything above the register layer is pinned
host-side by the promoted baseline tests
(`crates/bsw-can/tests/baseline_stm32_can_semantics.rs`,
`crates/bsw-bsp-stm32/tests/baseline_stm32_can_semantics.rs`), and the
seam-level contracts equivalent to these gaps remain pinned host-side
where they exist.

- **Gap 33** — upstream: bxCAN init-mode handshake is a bounded busy-wait
  (`INIT_TIMEOUT_CYCLES`); `init()`/`start()` return false on timeout,
  mapped to INIT_FAILED with all interrupts left masked. Port: FDCAN side
  already bounded (`MAX_INIT_POLLS` -> `InitFailed`); bxCAN
  `enter_init_mode`/`leave_init_mode` wait unbounded on INAK. Status:
  deferred — bounding the bxCAN handshake is a production-driver change
  requiring HIL re-verification of the init path.
- **Gap 34** — upstream: `start()` drains stale FIFO frames (bounded
  snapshot) and clears FOVR0 before enabling the RX interrupt. Port: bxCAN
  `open()` enables FMPIE0 without a stale-FIFO drain or overrun-flag
  clear. Status: deferred — the drain/clear sequence touches the
  production RX bring-up path; not host-testable without a new
  register-level seam.
- **Gap 35** — upstream: FDCAN RX ISR drains a fill-level snapshot so a
  busy bus cannot hold the ISR in an endless loop. Port:
  `can_fdcan::isr_rx_fifo0` loops until the live fill level reads 0.
  Status: deferred — ISR restructure of the production FDCAN driver;
  bounded-drain behavior needs target verification under real bus load.
- **Gap 36** — upstream: RF0R is rc_w1; the release write must be a direct
  write, never read-modify-write. Port: `can_bxcan::receive_isr` releases
  RFOM via PAC `modify()` (RMW), which can silently consume a newly
  latched FOVR edge. Status: deferred — production ISR register-access
  change requiring HIL verification of the overrun path. The equivalent
  seam-level contract IS pinned host-side (comparison row 11:
  `can_health::service_registers` single-snapshot write-back, test
  `fdcan_ir_acknowledge_is_single_snapshot_write_back`).
- **Gap 37** — upstream: user mute wins over bus-off auto-recovery
  (`fMuted` gates MUTED -> OPEN). Port: bxCAN/FDCAN timed recovery
  re-runs init/open unconditionally, so a user-muted transceiver would be
  silently unmuted by recovery. Status: deferred — policy nuance inside
  the production recovery sequence; adopting it changes driver recovery
  flow and requires HIL bus-off/mute interaction tests on both boards.
