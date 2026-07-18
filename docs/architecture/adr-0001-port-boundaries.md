# ADR-0001: Port architecture boundaries

- Status: accepted
- Date: 2026-07-17
- Applies from: packages A08 and B03-B06

## Context

OpenBSW combines portable libraries, RTOS integrations, platform adapters, and
a composed reference application. A literal crate-per-C++-target translation
would preserve dependency accidents and make `no_std` and host testing harder.

## Decision

Portable protocol and utility crates are independent of platform crates and
default to `no_std`. A `std` feature may add host adapters or diagnostics but
must not change protocol behavior. Portable production code does not depend on
`bsw-bsp-stm32`, a PAC/HAL, an RTOS, the POSIX adapter, or application crates.

Protocol crates do not allocate. Fixed capacities are explicit types or
configuration constants; pool exhaustion and output overflow are typed error
paths. `alloc`, `Vec`, `Box`, `String`, and heap-backed collections are
prohibited unless a later ADR identifies a non-protocol host-only crate and a
feature-gated use.

Time enters portable code through monotonic clock/instant/deadline traits.
Protocol code must not read DWT, SysTick, an OS clock, or wall time directly.

Fallible public boundaries return typed `Result` values. Panics are limited to
documented programmer-contract violations, infallible compile-time capacity
assumptions, and internal invariant failures. Bytes, frames, network packets,
diagnostic requests, generated configuration, and HIL input are untrusted and
must not cause a panic.

Configuration is validated at composition time and passed as typed immutable
data. Board pin, clock, interrupt, flash, and peripheral ownership belongs to a
single platform composition layer. Application examples may request services;
they may not duplicate MMIO, ISO-TP, UDS, storage, or scheduler implementations.

Crate dependencies flow in this direction:

```text
application/configuration -> platform adapters -> protocol/services -> common utilities
```

Callbacks must use bounded registration or static composition. Cyclic crate
dependencies, platform types in protocol APIs, hidden global clocks, and
application-specific behavior in protocol crates are prohibited.

## Consequences

Some upstream classes become native Rust traits or fixed-capacity structures
rather than direct translations. Such a row is complete only after the parity
manifest says `native replacement` and links behavioral evidence. `excluded`
is a scope decision, not successful implementation.
