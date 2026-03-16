# Plan: Complete Rust OpenBSW Stack + HIL Verification

## Context

openbsw-rust has 12 crates (826 tests), HW-verified BSP, and real-bus CAN (5/5 UDS PASS both directions). But examples use **inline** ISO-TP/UDS — the crate-level `bsw-docan`, `bsw-uds`, `bsw-lifecycle` aren't wired to hardware. Goal: replace inline code with real crate stack, add missing features, verify on HW via Pi HIL.

## Phases

```
Phase 1 (Wire BSW Stack) ──┬──► Phase 2 (Interrupt CAN RX)
                            ├──► Phase 6 (E2E)
Phase 3 (Flash/NvM) ───────┼──► Phase 4 (More UDS)
                            │            ▼
                            │     Phase 5 (DEM)
                            └──► Phase 7 (HIL)
```

---

### Phase 1 — Wire Crate-Level BSW Stack

Replace inline ISO-TP + UDS with `bsw-docan` codec, `bsw-uds` DiagRouter, `bsw-lifecycle` LifecycleManager.

**Create**:
- `src/diag_can.rs` — `DiagCanTransport<T: CanTransceiver>`: owns transceiver + CodecConfig + TX/RX protocol handlers + reassembly buffer + DiagRouter + session. `poll()` method: read frame → `decode_frame()` → drive RxProtocolHandler → on complete → `DiagRouter::dispatch()` → encode response → drive TxProtocolHandler → write frames.
- `examples/bsw_stack_g474.rs` — LifecycleManager + DiagCanTransport + FdCanTransceiver
- `examples/bsw_stack_f413.rs` — same with BxCanTransceiver

**Modify**: `Cargo.toml` (add bsw-docan/uds/lifecycle/transport/util, all `default-features = false`), `lib.rs` (add module)

**Verify**: Pi `cansend can0 600#023E00` → `candump` shows `601#027E00`

---

### Phase 2 — Interrupt-Driven CAN RX

**Create**: `src/can_isr.rs` — `#[interrupt] fn FDCAN1_IT0()` / `fn CAN1_RX0()`, static `spsc::Queue<CanFrame, 33>`

**Modify**: `can_fdcan.rs` (add `enable_rx_interrupt()`, `receive()` reads from SPSC), `can_bxcan.rs` (same)

**Verify**: Pi sends 20-frame burst, zero drops (check via UDS ReadDID for statistics)

---

### Phase 3 — Flash/NvM Driver

**Create**:
- `src/flash_g4.rs` — unlock, erase_page(4KB), program_doubleword, lock. Reserve pages 124-127 (0x0807_C000)
- `src/flash_f4.rs` — F413 sector-based variant
- `src/nvm.rs` — `NvmManager`: block table (VIN 17B, serial 10B, DTCs 256B, cal 64B), CRC32 integrity via `bsw-util::crc`

**Verify**: Write VIN, power cycle, read back — persists

---

### Phase 4 — More UDS Services

Add to `bsw-uds/src/services.rs`:

| SID | Service | Key detail |
|-----|---------|-----------|
| 0x2E | WriteDID | NvM write, needs security + extended session |
| 0x11 | ECUReset | Deferred `SCB::sys_reset()` after TX |
| 0x27 | SecurityAccess | Seed/key XOR+rotate, 3-attempt lockout |
| 0x31 | RoutineControl | 0xFF00=LED test, 0xFF01=CAN self-test, 0xFF02=uptime |
| 0x19 | ReadDTCInfo | Delegates to DEM (Phase 5) |

**Verify**: From Pi: TesterPresent → ExtendedSession → SecurityAccess → WriteDID(VIN) → ReadDID(VIN) → ECUReset

---

### Phase 5 — DEM (Diagnostic Event Manager)

**Create**: `bsw-uds/src/dem.rs` — `DemManager<MAX_DTCS>`: DtcEntry (code, status_byte, counters), `report_event()`, `clear_all()`, NvM persistence

DTCs: 0xC07300 (bus-off), 0xD00100 (watchdog), 0xD10000 (flash fail)

**Verify**: Disconnect CAN_H → reconnect → ReadDTCInfo → 0xC07300 confirmed. ClearDTC → empty.

---

### Phase 6 — E2E Protection

**Create**: `bsw-util/src/e2e.rs` — `E2eProtector::protect()`, `E2eChecker::check()` using CRC8 SAE J1850 + alive counter

**Verify**: Pi sends correct E2E → accepted. Bad CRC → rejected. Stale counter → detected.

---

### Phase 7 — HIL Verification

**Create**: `openbsw-rust/hil/` — Python test suite on Pi

- `hil_runner.py` — orchestrator using `python-can`
- `can_helpers.py` — ISO-TP encode/decode, UDS helpers
- `scenarios/*.yaml` — ~25 test scenarios

Categories: basic UDS (5), security (3), multi-frame (2), persistence (2), DTC lifecycle (3), E2E (3), negative paths (4), ECUReset (1)

**Run**: `ssh pi@192.168.0.195 "cd openbsw-rust/hil && python3 hil_runner.py"` → all green

---

## Execution Order

| Step | Phase | Parallel? |
|------|-------|-----------|
| 1 | Phase 1 (wire BSW stack) | + Phase 3 (flash/NvM) |
| 2 | Phase 2 (interrupt CAN RX) | + Phase 6 (E2E) |
| 3 | Phase 4 (more UDS) | sequential |
| 4 | Phase 5 (DEM) | sequential |
| 5 | Phase 7 (HIL) | final |

Each phase: implement → `cargo test` → cross-compile → flash → HW verify → commit → push.
