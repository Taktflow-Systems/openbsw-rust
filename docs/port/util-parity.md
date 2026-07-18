# `util` parity decisions

Baseline: OpenBSW `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`,
`libs/bsw/util`. The table classifies every production include/source area and
records the dependency decision used by downstream Rust modules.

| Upstream area | Rust decision | Owner/evidence |
|---|---|---|
| `crc` | ported, including upstream parameterization and golden vectors | `bsw_util::crc`, `crc_oracle` |
| `spsc` | ported fixed-capacity atomic SPSC queue | `bsw_util::spsc`, controlled concurrency tests |
| `memory/Bit` | native integer bit operations | compiler |
| `memory/BuddyMemoryManager` | ported | `bsw_util::buddy` tests |
| `math/MovingAverage` | ported | `bsw_util::moving_average` tests |
| `types/Enum` | native replacement: Rust enums plus `From`/`TryFrom` | compiler |
| `meta/Bitmask` | native replacement: newtypes with bit operators | compiler |
| `meta/BinaryValue` | native replacement: typed constants and `const fn` | compiler |
| `defer/Defer` | native replacement: RAII and `Drop` guards | compiler |
| `string/ConstString` | native replacement: `&'static str`; bounded mutable text uses `BoundedString` | estd tests |
| `format/*` | native replacement: `core::fmt`, `format_args!`, `fmt::Write`; bounded sinks are C13/C14 | formatting tests |
| `preprocessor/Macros` | native replacement: declarative macros and `cfg` | compiler |
| `estd/assert` | native replacement: `assert!` for programmer invariants and typed errors for external input | panic policy |
| `estd/derived_object_pool`, `block_pool` | `ObjectPool`/fixed buffers; specialized transport pools remain E01 | estd parity and E01 |
| `estd/intrusive`, `signal`, `va_list_ref` | `ForwardList`, explicit callbacks/atomics, and typed format arguments | estd parity/C13 |
| `buffer/LinkedBuffer` | assigned to bounded blob/transport buffers | D11/E01 |
| `stream/*` | assigned to common bounded sinks and POSIX/embedded adapters | C14/C17/C18 |
| `logger/*` | assigned to allocation-free logging | C13/C14 |
| `command/*` | assigned to console registry and parser | C15-C18 |

No downstream package needs to port an unassigned utility dependency. Items
not implemented in this tranche have an explicit later package and may not be
re-created ad hoc in protocol crates.
