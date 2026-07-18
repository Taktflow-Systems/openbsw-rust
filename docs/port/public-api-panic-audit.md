# Public API panic audit

Policy comes from ADR-0001: untrusted bytes and hardware/network/HIL input must
return typed errors; panics are reserved for programmer contract violations or
broken internal invariants.

The 2026-07-17 audit found these production panic families:

| Area | Current classification | Planned owner |
|---|---|---|
| `bsw-can::CanFrame` payload capacity | external input; typed `FrameError` APIs added in B04, panicking convenience APIs retained for trusted callers | B04/D12 |
| COM pack/unpack bounds | external descriptors/input; migrate to typed errors | E32 |
| BSP GPIO pin, timer frequency, NvM descriptor | composition/programmer configuration; validate in typed board configuration | G01/G07/D09 |
| `bsw-estd` index operations and map indexing | Rust collection-style programmer contract; keep panicking forms and provide fallible forms where absent | C02-C04 |
| `bsw-estd`/`bsw-io` const-capacity assertions | compile-time/type invariant | C02-C04/D04 |
| object pool unreachable branch | internal invariant; retain only with invariant tests | C04 |
| buddy allocator geometry assertions | construction/programmer contract; prefer const validation | C05 |
| E2E buffer-length assertions | external buffer; migrate to typed errors | E34 |

B04's first migration adds `try_with_data`, `try_with_raw_id`,
`try_set_payload`, and `try_set_payload_length`. Both hardware RX paths use the
fallible constructor, so malformed or future DLC handling cannot cross the
public frame boundary by panicking. Existing panicking methods remain source
compatible and explicitly document their contract.
