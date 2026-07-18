# COM extension status - packages E32-E33

The pinned Eclipse OpenBSW baseline has no general COM signal codec, cyclic or
event scheduler, or receive-deadline monitor. `bsw-com` is therefore a project
extension, not an upstream or AUTOSAR-conformance claim.

| Behavior | Rust implementation | Evidence |
|---|---|---|
| Intel and DBC-sawtooth Motorola bit packing | `packer` | exhaustive widths, hand-computed byte images, and property tests |
| Signed fields and integer scaling | `Scaling`, physical pack/unpack | boundary, overflow, and rounding tests |
| CAN FD payloads through 64 bytes | `PDU_BUFFER_LEN` | upper-payload TX/RX and property tests |
| Drift-free cyclic TX | `ComManager::start` / `tick` | exact-boundary, late-tick, overrun, and wraparound scenarios |
| EventOnly and Mixed TX | `TxMode`, `TransferProperty` | off-grid, fold-at-grid, ordering, and pending-property scenarios |
| Minimum event interval | `PduDescriptor::min_event_interval` | throttling and one-event-per-pending-request scenario |
| RX deadlines and timeout defaults | `RxTimeoutAction` | first deadline, re-arm, exact boundary, RestoreInit, KeepLast, and recovery scenarios |
| Update and invalid flags | `take_updated`, `is_signal_valid` | TX clear-on-yield and RX read-and-clear scenarios |
| Bounded error reporting | `ComEvent` FIFO | drop-newest capacity and saturating drop counter scenario |

All platform-independent state is fixed-capacity, heap-free, and driven by
injected `bsw_time::Instant` values. The extension passes all-feature and
no-default-feature tests, clippy with warnings denied, the COM fuzz-target
build, and the workspace all-feature test gate.
