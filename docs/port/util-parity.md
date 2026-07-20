# `util` parity decisions

Baseline: OpenBSW `be0029bbb79fe901048a24c2665f2ba854328734` (re-pinned
2026-07-20 from `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`; see
`docs/port/repin-2026-07-20.md`), `libs/bsw/util` as it stood at the
original baseline. The table classifies every production include/source area
and records the dependency decision used by downstream Rust modules. Rows
whose upstream area was removed at the tip in favor of ETL carry a
pin-to-tip marker; all other areas were spot-checked as still present at
`be0029b`.

| Upstream area | Rust decision | Owner/evidence |
|---|---|---|
| `crc` | ported, including upstream parameterization and golden vectors | `bsw_util::crc`, `crc_oracle` (pin-to-tip: util `crc` removed upstream in favor of etl crc types, e.g. `crc8_ccitt`/`crc32`, `c1d18f8d`; native replacement unaffected) |
| `spsc` | ported fixed-capacity atomic SPSC queue | `bsw_util::spsc`, controlled concurrency tests (pin-to-tip: util `spsc` removed upstream in favor of etl `queue_spsc_atomic`, `db24855a`/`5c0dedaf`; native replacement unaffected) |
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
| `estd/assert` | native replacement: `assert!` for programmer invariants and typed errors for external input | panic policy (pin-to-tip: util `estd/assert` removed upstream in favor of etl `ETL_ASSERT`/`ETL_ERROR_GENERIC`, `eb956887`; native replacement unaffected) |
| `estd/derived_object_pool`, `block_pool` | `ObjectPool`/fixed buffers; specialized transport pools remain E01 | estd parity and E01 (pin-to-tip: both removed upstream in favor of etl `pool`/`ipool`/`generic_pool`, `eb956887`; native replacement unaffected) |
| `estd/intrusive`, `signal`, `va_list_ref` | `ForwardList`, explicit callbacks/atomics, and typed format arguments | estd parity/C13 (pin-to-tip: `intrusive` and `signal` removed upstream in favor of etl `intrusive_forward_list` and delegate facilities, `eb956887`; `va_list_ref.h` still present at the tip; native replacements unaffected) |
| `buffer/LinkedBuffer` | assigned to bounded blob/transport buffers | D11/E01 |
| `stream/*` | assigned to common bounded sinks and POSIX/embedded adapters | C14/C17/C18 |
| `logger/*` | assigned to allocation-free logging | C13/C14 |
| `command/*` | assigned to console registry and parser | C15-C18 |

No downstream package needs to port an unassigned utility dependency. Items
not implemented in this tranche have an explicit later package and may not be
re-created ad hoc in protocol crates.

## Re-pin 2026-07-20: native-replacement re-review at `be0029b`

**estd/util removals, StringBufferOutputStream, TimestampProvider (survey
CV-1, `9d3e89a2`).** The port's native replacements (rows C02-C08, C13-C14,
and the G05/D01-D02 time surface, whose parity rows live in
`docs/port/stm32-parity.md` and the parity manifest) are re-affirmed at the
tip: no port code is shared with upstream `estd`/`util`, so upstream's
removals in favor of ETL do not invalidate any counterpart mapping. The
upstream `StringBufferOutputStream` finalization/destructor fix
(`07b75511`, `8f31f107`) hardens a C++ destructor-ordering hazard that has
no Rust counterpart — the port's bounded sinks use `core::fmt::Write` over
owned buffers with no finalize-on-destroy step. The
TimestampProvider/SystemTimer rework (`9d3e89a2`) reshapes upstream's C++
time API; the port's `bsw-time` (u64-nanosecond `Instant`) is a native
replacement with no shared surface. The removed-area rows above carry
their individual pin-to-tip markers.

**Console stdin/UART robustness (survey CV-2, `bd0078d0`, `f381dd46`).**
The console parity rows C15-C18 live in this document (`command/*` and
`stream/*` rows above). The upstream commits harden upstream's POSIX stdin
handling: `getByteFromStdin` checked the size of a span passed by value
instead of the `read()` return, and `Uart::read` silently converted a
`-1` return into a large `size_t`. The port's console implementation
(`crates/bsw-console/src/posix.rs`) is unaffected: it reads through
`std::io::Read`, which returns `io::Result<usize>` — errors are typed,
EOF is `Ok(0)`, and the signed-to-unsigned conversion bug class is
structurally impossible. No port change is needed.

**EEPROM-size configurability (`1578c18f`).** Storage parity rows
(D06-D11, G11) live in `docs/port/parity-manifest.json` /
`docs/port/status.md`; the disposition is recorded here as the assigned
re-pin doc. Upstream moved the hardcoded POSIX emulated-EEPROM size
(`EEPROM_SIZE = 4096` in `EepromDriver.h`) into the integrator-owned
`bspConfiguration/include/bsp/EepromConfiguration.h`; the default value
and default-build behavior are unchanged. The Rust port already expresses
storage size as integrator configuration: `bsw_storage::mem::MemBackend<
SIZE, ERASE, PROG>` takes the region size as const generics,
`bsw_storage::file::FileBackend` takes a runtime `FileGeometry
{ region_size, erase_unit, program_unit }`, and the reference app pins its
own geometry in its composition (`crates/openbsw-reference-app/src/
storage.rs`, `AppBackend = MemBackend<16_384, 1_024, 4>`) rather than in
the driver crate. Disposition: default behavior identical, configurability
native and already expressible; no port change is needed.
