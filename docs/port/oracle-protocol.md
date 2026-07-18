# Differential oracle protocol, version 1

The pinned C++ application and the Rust port exchange newline-delimited or
document-wrapped JSON records. A document has this shape:

```json
{"protocol_version":1,"scenario":"can-frame-smoke","records":[...]}
```

Every record contains `sequence` (monotonic within the scenario), `kind`,
`source`, `input`, `output`, `state`, and optional `timestamp_ns` and `logs`.
Byte strings use lowercase hexadecimal without separators; CAN IDs are JSON
numbers; durations are integer nanoseconds. Object keys are semantically
unordered. Arrays, sequence numbers, inputs, outputs, and state transitions
are ordered and significant.

Before comparison, the harness removes `source` and `timestamp_ns`. Logs are
compared only in scenarios that declare `compare_logs: true`; otherwise state
and externally visible output are the contract. No pointer, process ID, wall
clock, random seed, absolute path, or host-specific interface name may appear
in normalized records.

## Scenario contract

Each scenario records:

- a stable scenario name and protocol version;
- the exact upstream commit and Rust repository revision when captured;
- deterministic input records and an explicit initial state;
- output records plus terminal state;
- normalization rules and any approved intentional difference.

The bootstrap fixture at `docs/port/oracle-fixtures` exercises the comparison
contract with CAN ID classification. It is protocol evidence, not a claim that
CAN parity is complete. Later packages must replace bootstrap provenance with
records emitted by the built C++ and Rust applications.

Run the deterministic smoke comparison with:

```powershell
python tools/port/compare_oracle.py docs/port/oracle-fixtures/can-frame-openbsw.json docs/port/oracle-fixtures/can-frame-rust.json
```

Protocol changes require a new integer version and retained readers/fixtures
for evidence that remains part of a release record.
