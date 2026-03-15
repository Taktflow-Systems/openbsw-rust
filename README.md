# openbsw-rust

Incremental Rust rewrite of [Eclipse OpenBSW](https://github.com/eclipse-openbsw/openbsw) — an open-source AUTOSAR-inspired BSW stack.

## Methodology: Wrap, Replace, Prove

This is **not** a one-shot translation. We follow the incremental strangler-fig pattern:

```
1. Freeze behavior    — tests, golden outputs, protocol traces
2. Bridge one module  — cxx/autocxx boundary, C++ still does the work
3. Rewrite inward     — AI drafts logic, human reviews FFI/unsafe
4. Prove equivalence  — old C++ stays as oracle, both run same tests
5. Shrink C++         — remove C++ once Rust passes all checks
6. Repeat
```

### Rules

- **No big-bang rewrites** — one module at a time, system works at every step
- **AI writes logic, not boundaries** — FFI/unsafe blocks need human review + written safety justification
- **Old C++ is the oracle** — Rust must match C++ behavior exactly before C++ is removed
- **Leaf modules first** — pure logic, no HW, no OS, no protocol state machines
- **Every `unsafe` block must have a `// SAFETY:` comment** explaining the invariant

### Module Priority

| Phase | Module | LOC | Complexity | Status |
|-------|--------|-----|------------|--------|
| 1 | `timer` | 259 | Pure logic, sorted list | Done (in new-era-toolkits) |
| 1 | `async` | 140 | Call/delegate wrappers | Done (in new-era-toolkits) |
| 2 | `util` | 14,410 | Mixed — format, stream, string, command | Planned |
| 2 | `estd` | 9,395 | Template containers (array, optional, variant) | Planned |
| 3 | `storage` | 1,923 | NvM-like persistence | Planned |
| 3 | `lifecycle` | 2,020 | Component init/shutdown state machine | Planned |
| 3 | `logger` | 3,507 | Logging framework | Planned |
| 4 | `transport` | 2,222 | Transport message framing | Planned |
| 4 | `cpp2can` | 3,603 | CAN bus abstraction | Planned |
| 4 | `cpp2ethernet` | 3,942 | Ethernet abstraction | Planned |
| 5 | `docan` | 16,320 | CAN transport protocol (ISO 15765) | Planned |
| 5 | `uds` | 18,510 | Diagnostic protocol (ISO 14229) | Planned |
| 5 | `doip` | 19,744 | Diagnostics over IP (ISO 13400) | Planned |

### Tooling

| Tool | Role |
|------|------|
| `cxx` / `autocxx` | Safe Rust-C++ interop boundary |
| `cargo test` | Rust-side tests |
| `googletest` | C++ oracle tests (run both, compare) |
| `clippy` | Lint |
| `miri` | UB detection in unsafe blocks |
| Sonnet | Module-level rewrites (pure logic) |
| Opus | Architecture decisions, FFI boundary design |

### Workspace Structure

```
openbsw-rust/
├── Cargo.toml              # workspace root
├── crates/
│   ├── bsw-timer/          # timer module (Phase 1)
│   ├── bsw-async/          # async utilities (Phase 1)
│   ├── bsw-util/           # util module (Phase 2)
│   ├── bsw-estd/           # estd containers (Phase 2)
│   ├── bsw-transport/      # transport framing (Phase 4)
│   ├── bsw-docan/          # CAN transport (Phase 5)
│   ├── bsw-uds/            # UDS diagnostics (Phase 5)
│   └── bsw-doip/           # DoIP (Phase 5)
├── bridge/                 # cxx/autocxx FFI boundaries
│   └── docan-bridge/       # example: docan C++ ↔ Rust bridge
└── tests/
    └── oracle/             # comparison tests: C++ vs Rust output
```

## Origin

Based on [Eclipse OpenBSW](https://github.com/eclipse-openbsw/openbsw) (Apache-2.0).
Phase 1 modules (timer, async) prototyped in [new-era-toolkits](https://github.com/nhuvaoanh123/new-era-toolkits).

## License

Apache-2.0 (matching upstream OpenBSW)
