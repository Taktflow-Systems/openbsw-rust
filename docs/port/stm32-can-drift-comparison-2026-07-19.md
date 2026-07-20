# STM32 CAN drift comparison — 2026-07-19 (package U03)

Follow-up to `docs/port/upstream-drift-2026-07-18.md`. The pinned oracle
remains `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77` and **was not moved**; the
drift tip analyzed is `be0029bb`. All upstream reading was done against the
ignored read-only bare checkout under `target/drift/`; **no upstream platform
code was adopted**, no existing contract or production code was changed, and
`docs/port/parity-manifest.json` / `status.md` / release and parity evidence
documents are untouched. This memo plus two additive drift-labeled test files
are the entire output of the package.

## Commits reviewed (9, CAN/STM32 area of the drift assignment)

| Commit | Subject | Relevance |
|---|---|---|
| `1a11d135` | Add STM32 CAN drivers and bxCAN transceiver | primary: `BxCanDevice`, `FdCanDevice`, `BxCanTransceiver` semantics |
| `be0029bb` | Add STM32 FDCAN transceiver adapter | primary: `FdCanTransceiver` semantics (also the drift tip) |
| `cf3832d5` | Add STM32 BSP peripheral drivers | context: clock/GPIO/UART/timer/ETL-impl foundation |
| `b581e80a` | Add STM32 MCU foundation | context: startup, linker, MCU headers scaffold |
| `56af88ec` | Add STM32 device headers via RIM | context: vendored ST headers (not adopted here) |
| `6cafa673` | Fix logger to work for extended canId | semantic: display/log path must use the raw arbitration id |
| `2f3e19e0` | Add STM32 unit-test builds to CI | process: host unit-test builds for the STM32 CAN tests |
| `f2f01102` | Fix STM32 CAN unit tests on 64-bit hosts | semantic: 32-bit register mask semantics on LP64 hosts; delegate lifetime |
| `830b9c5e` | Fix removed SystemTimer calls in STM32 ETL clock implementation | semantic: 64-bit timestamp API for the STM32 clock impl |

Upstream sources read in full: `platforms/stm32/bsp/bspCan/` (BxCanDevice,
FdCanDevice + tests), `platforms/stm32/bsp/bxCanTransceiver/`,
`platforms/stm32/bsp/fdCanTransceiver/`, and the diffs of the four fix/process
commits. Rust counterparts compared: `crates/bsw-can/src/{transceiver,
error_state, filter, can_id, dlc, listener}.rs` and
`crates/bsw-bsp-stm32/src/{can_bxcan, can_fdcan, can_health, can_isr}.rs`.

## Semantic comparison

Classification: **same** (same observable semantics), **stronger** (Rust
guarantees strictly more), **design** (different by documented design,
upstream-observable behavior preserved at the intended layer), **gap**
(upstream demonstrates behavior the Rust port lacks; classified testable-now
vs re-pin-dependent).

### Same semantics (drift tests added where marked T)

| # | Upstream behavior (commit) | Rust location | Class |
|---|---|---|---|
| 1 | `init()` only from CLOSED, else ILLEGAL_STATE (`1a11d135`, `be0029bb`) | `AbstractTransceiver::transition_to_initialized`; both drivers guard | same, T |
| 2 | `mute()` only from OPEN, `unmute()` only from MUTED | same guards in trait impls | same, T |
| 3 | `close()` from CLOSED returns OK (idempotent) | `transition_to_closed` always Ok | same, T |
| 4 | bus-off supervision restores MUTED to OPEN on recovery | `transition_to_open` accepts Muted | same, T |
| 5 | write while not OPEN or muted returns TX_OFFLINE | `TxOffline` in `can_bxcan::write` / `can_fdcan::write` | same (vocabulary pinned by T; behavior target-verified in G20) |
| 6 | TX hardware queue full: overrun counter + TX_HW_QUEUE_FULL | `statistics.tx_dropped` + `TxHwQueueFull` | same, T |
| 7 | classic wire DLC 9..15 clamped to 8 bytes | drivers clamp; `bsw_can::dlc::dlc_to_length` | same, T |
| 8 | ISR bit-field filter is standard-ID-only; extended frames bypass it | ISR accepts all; `BitFieldFilter` domain 0..=0x7FF, out-of-range never matches | same, T |
| 9 | log/display path uses the raw arbitration id, not the encoded word (`6cafa673`) | `CanId::raw_id()` vs `CanId::value()` | same, T |
| 10 | FDCAN M_CAN has no automatic bus-off recovery; caller must re-init after its delay policy (`be0029bb`) | `can_fdcan::service_health`: explicit close/init/open after exact injected deadline; `ErrorStateTracker` latches until `begin_recovery` + deadline | same, T |
| 11 | rc_w1 registers cleared with a direct write of the read snapshot, never read-modify-write (`1a11d135`, `f2f01102`) | `can_health::service_registers` FDCAN path: one snapshot read, one write-back; bxCAN path: zero writes | same at the health seam, T (see gap 27 for the bxCAN driver deviation) |
| 12 | bus-off detected from ESR.BOFF / PSR.BO status bits | `CanInterruptFlags::from_bxcan/from_fdcan_ir`; `transceiver_state()` | same, T |
| 13 | bounded RX ISR drain via fill-level snapshot (bxCAN) | `can_bxcan::receive_isr` snapshots `FMP0.min(3)` | same (bxCAN only; see gap 26 for FDCAN) |
| 14 | STM32 clock impl uses 64-bit timestamps (`830b9c5e`) | `bsw-time` Instant is u64 ns; DWT wrap tests (G05) | same intent, native mechanism |
| 15 | STM32 unit tests build in CI (`2f3e19e0`) | workspace test gate, `check_features.py`, miri on `can_isr` | same intent, process-level |

### Rust stronger

| # | Upstream behavior | Rust behavior | Class |
|---|---|---|---|
| 16 | RX queue full: FIFO entry released without storing; drop is silent | `InterruptQueue` drop-newest **plus** cumulative `dropped()` counter and `statistics.rx_dropped` | stronger, T |
| 17 | RX batch drain requires masking the RX interrupt around count/clear | lock-free SPSC ring with release/acquire publication; no mask window needed | stronger (miri-checked) |
| 18 | adapter exposes only `isBusOff()`; error-passive visible only via raw counters | error-passive flag mapped into `TransceiverState::Passive` via `CanHealth` | stronger, T |
| 19 | `f2f01102` had to repair 32-bit mask semantics on LP64 hosts | `u32` typing makes the truncation class structurally impossible | stronger by construction |
| 20 | `f2f01102` had to repair a dangling `etl::delegate` bound to a temporary | ownership/borrow checking rejects the pattern at compile time | stronger by construction |

### Different by design (documented; upstream-observable behavior preserved at the intended layer)

| # | Upstream behavior | Rust design | Where documented |
|---|---|---|---|
| 21 | `open()` also accepted from CLOSED with internal re-init | explicit `init()` required; Closed -> Open rejected (T pins the rejection) | driver doc comments; this memo |
| 22 | `open(frame)`: wake-up frame unsupported, silently ignored | `open_with_frame` opens then actually transmits the frame | `bsw-can` trait docs |
| 23 | `mute()` is a software TX gate (flag); pending listener jobs dropped silently | hardware silent mode (bxCAN SILM / FDCAN equivalent), hardware-enforced listen-only | driver doc comments |
| 24 | `getHwQueueTimeout` computed as ceil(3x117x1000/baud) ms, never 0 (1 ms at 500k) | fixed 10 ms constant (conservative upper bound of the computed value) | `can_bxcan`/`can_fdcan` constants |
| 25 | deferred sent-listener TX job queue (depth 3 = HW depth, drop-without-notify on close/mute, ISR-chained sends) | no deferred TX-listener path; TX confirmation is a `FrameDispatcher` instance (native replacement) | `docs/port/can-parity.md` (ICANFrameSentListener rows) |
| 26 | bus-off folds into the logical state (OPEN -> MUTED), user-mute flag gates auto-reopen | bus-off modeled in `TransceiverState` (Active/Passive/BusOff); logical `State` unchanged | `can_health` module docs |
| 27 | bxCAN relies on hardware ABOM auto-recovery | ABOM deliberately disabled; deterministic 1 s timed recovery via RCC reset + re-init (target-proven in G20) | `docs/port/stm32-parity.md` |
| 28 | raw TEC/REC counters readable (FDCAN REC is 7-bit) | ISO thresholds modeled by `ErrorStateTracker` (128/256) instead of raw reads | `docs/port/can-parity.md` (fault confinement row) |
| 29 | hardware filter lists: bxCAN 14 banks x 2 std IDs (odd count duplicates last), FDCAN 28 exact-match elements with reject-non-matching, surplus ignored | hardware accept-all + full 2048-bit software filter; no surplus truncation | `docs/port/can-parity.md` |
| 30 | software RX queue stores 32 frames | SPSC ring reserves one slot: `InterruptQueue<N>` stores N-1 (F413: 31) (T pins the capacity) | `can_isr` module docs; this memo |
| 31 | TX-empty interrupt enabled per transmit, disabled when all mailboxes idle | bxCAN enables TMEIE at `open()` and leaves it on (scheduling policy, not an observable frame-level difference) | this memo |
| 32 | vendored ST device headers; register-level C drivers | typed Rust BSP (`board.rs`/`mmio.rs`), no vendor headers adopted (`b581e80a`, `cf3832d5`, `56af88ec`) | `docs/port/stm32-parity.md` G01/G02 |

### Gaps: upstream demonstrates behavior the Rust port lacks (all re-pin-dependent)

None of these are host-testable without new register-level fakes for the
production drivers (which would be new production seams, out of scope for an
additive-tests package); all are recorded for the next re-pin decision.

| # | Upstream behavior (commit) | Rust state | Classification |
|---|---|---|---|
| 33 | bxCAN init-mode handshake is a bounded busy-wait (`INIT_TIMEOUT_CYCLES`); `init()`/`start()` return false on timeout, adapter maps to INIT_FAILED with all interrupts left masked (`1a11d135`) | FDCAN side already bounded (`MAX_INIT_POLLS` -> `InitFailed`); bxCAN `enter_init_mode`/`leave_init_mode` wait unbounded on INAK | gap, re-pin-dependent (bxCAN only) |
| 34 | `start()` drains stale FIFO frames (bounded snapshot) and clears FOVR0 **before** enabling the RX interrupt (`1a11d135`) | bxCAN `open()` enables FMPIE0 without a stale-FIFO drain or overrun-flag clear | gap, re-pin-dependent |
| 35 | FDCAN RX ISR drains a fill-level **snapshot** so a busy bus cannot hold the ISR in an endless loop (`1a11d135`) | `can_fdcan::isr_rx_fifo0` loops until the live fill level reads 0 | gap, re-pin-dependent |
| 36 | RF0R is rc_w1; the release write must be a direct write, never RMW ("a read-modify-write would write back any flag that happens to be set and silently clear it") (`1a11d135`) | `can_bxcan::receive_isr` releases RFOM via PAC `modify()` (RMW), which can silently consume a newly latched FOVR edge between the entry check and the release | gap, re-pin-dependent; the equivalent seam-level contract is pinned host-side (row 11) |
| 37 | user mute wins over bus-off auto-recovery (`fMuted` gates MUTED -> OPEN) (`1a11d135`, `be0029bb`) | bxCAN/FDCAN timed recovery re-runs init/open unconditionally; a user-muted transceiver would be silently unmuted by recovery | gap (policy nuance), re-pin-dependent |

### Observation found while writing the drift tests

`bsw_can::ErrorStateTracker` latches bus-off against `record_tx_error` /
`record_rx_error` but not against `record_tx_success` / `record_rx_success`
(a success while bus-off decrements TEC below 256 and re-derives Passive).
This is unreachable through the drivers — a bus-off controller cannot report
a successful transfer — so it is not a contract break, but it is weaker than
the upstream "latched until re-init" model at the API level. Recorded here;
no production change made (out of scope for this additive package).

## Classification counts

- same semantics: 15 (rows 1-15; 13 of them CAN-contract rows, 2 same-intent
  process/mechanism rows)
- Rust stronger: 5 (rows 16-20)
- different by design: 12 (rows 21-32)
- gap / upstream demonstrates behavior Rust lacks: 5 (rows 33-37), all
  re-pin-dependent, none testable on the host today without new production
  seams
- plus 1 API-level observation (ErrorStateTracker success-path latch)

## Drift-labeled contract tests added

Both files carry the required header: derived from the named upstream drift
commits, drift tip `be0029bb`, explicitly **not** pinned-oracle evidence.

`crates/bsw-can/tests/drift_stm32_can_semantics.rs` (6 tests, run by the
workspace gate via `cargo test -p bsw-can --all-features`):

- `lifecycle_guards_match_upstream_adapter_state_machine`
- `bus_off_is_latched_until_explicit_timed_recovery`
- `bit_field_filter_domain_is_standard_ids_only`
- `classic_dlc_nine_to_fifteen_clamp_to_eight_bytes`
- `extended_id_raw_value_strips_qualifier_bits_for_display`
- `tx_saturation_vocabulary_and_drop_accounting_exist`

`crates/bsw-bsp-stm32/tests/drift_stm32_can_semantics.rs` (5 tests, host-run
via the crate's existing host-test path `cargo test -p bsw-bsp-stm32`, the
same invocation family `tools/port/check_miri.ps1` uses; only the
feature-ungated `can_health`/`can_isr` seams are exercised, matching the
crate's convention that register-level driver code is target/HIL scope):

- `bxcan_status_decode_is_read_only`
- `fdcan_ir_acknowledge_is_single_snapshot_write_back`
- `rx_queue_full_drops_newest_preserves_order_and_accounts`
- `bus_off_supervision_uses_exact_deadline_and_monotonic_overflow_accounting`
- `error_passive_flag_maps_to_passive_state`

## Verification

- `cargo test -p bsw-can --all-features`: pass (95 lib + 37 + 6 drift + 1 doc)
- `cargo test -p bsw-bsp-stm32`: pass (25 lib + 5 drift)
- `python tools/port/check_stm32_examples.py`: pass (examples untouched)
- `cargo fmt --all -- --check`: clean for the authored files
- `python tools/port/check_privacy.py`: pass

## Explicit statements

- No upstream platform code was adopted; upstream sources were read from the
  ignored drift checkout only.
- No existing contract, production module, test, or evidence document was
  modified or weakened; the package is strictly additive (one memo, two new
  test files).
- The pinned oracle `ddbcf88a` was not moved; nothing under the oracle or
  parity manifest changed.
