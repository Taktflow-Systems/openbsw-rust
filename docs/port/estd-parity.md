# `estd` parity decisions

Baseline: OpenBSW `be0029bbb79fe901048a24c2665f2ba854328734` (re-pinned
2026-07-20 from `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`; see
`docs/port/repin-2026-07-20.md`), `libs/bsw/estd/include/estd` as it stood
at the original baseline. Every public upstream header is classified below.

Pin-to-tip note (2026-07-20): at `be0029b` upstream has removed the entire
`estd` library in favor of ETL (`eb956887` "Remove estd containers" and the
preceding migration commits; survey CV-1). The classification below keeps
the original header names because they are what the port's decisions were
made against; rows whose upstream type has a specific ETL successor at the
tip carry a pin-to-tip marker. No port code was ever shared with upstream
`estd`, so the removals do not invalidate any counterpart mapping — every
Rust decision below is unaffected.

| Upstream header/type | Rust decision | Evidence |
|---|---|---|
| `algorithm.h` | native replacement: iterator/slice methods | workspace tests |
| `array.h` | native replacement: `[T; N]` and `core::array` | compiler/runtime tests |
| `big_endian.h` | ported | `bsw_estd::big_endian` tests (pin-to-tip: estd `big_endian` removed upstream in favor of etl `be_uint16_t`/`be_uint32_t`/`be_uint64_t` unaligned types; native replacement unaffected) |
| `bitset.h` | ported | `bsw_estd::Bitset` tests (pin-to-tip: estd `bitset` removed upstream in favor of etl `bitset`; native replacement unaffected) |
| `constructor.h` | native replacement: `MaybeUninit`, `Default`, placement through pool APIs | Miri container suite |
| `forward_list.h` | ported with direct node pointers and explicit `clear` teardown | `forward_list` tests and Miri (pin-to-tip: estd `forward_list` removed upstream in favor of etl `intrusive_forward_list`; native replacement unaffected) |
| `functional.h` | native replacement: `Fn`, `FnMut`, `FnOnce` | compiler |
| `iterator.h` | native replacement: `Iterator` traits | container tests |
| `memory.h` | native replacement: references, `Box` only in `std` ownership domains | allocation policy |
| `none.h` | native replacement: `None` and unit `()` | compiler |
| `object_pool.h` | ported; zero-sized types are rejected | `object_pool` tests and Miri (pin-to-tip: estd `object_pool` removed upstream in favor of etl `pool`/`ipool`/`generic_pool`; native replacement unaffected) |
| `optional.h` | native replacement: `Option<T>` | compiler |
| `ordered_map.h` | ported | `OrderedMap` tests and Miri (pin-to-tip: estd `ordered_map` removed upstream in favor of etl `map`/`imap`; native replacement unaffected) |
| `slice.h` | native replacement: `&[T]` / `&mut [T]`; bounded byte subviews use `bsw_common::ByteView` | common tests |
| `type_list.h` | native replacement: tuples, generics, macros | compiler |
| `type_traits.h` | native replacement: trait bounds and marker traits | compiler |
| `type_utils.h` | native replacement: associated types and `core::mem` | compiler |
| `uncopyable.h` | native replacement: absence of `Copy`/`Clone` implementations | compiler |
| `variant.h` | native replacement: enums | compiler |
| `vector.h` | ported as `FixedVec<T, N>` | vector tests, Miri, fuzz (pin-to-tip: estd `vector` removed upstream in favor of etl `vector`; native replacement unaffected) |

`BoundedString<N>` is an additional inline UTF-8 primitive required by later
console/logging work. It preserves UTF-8 boundaries, reports capacity failure,
and supports zero capacity. It does not replace an unclassified `estd` header.

The deliberate semantic differences are ownership improvements: intrusive
lists encode node lifetime in `ForwardList<'a, T>`, store whole-node pointers,
and require explicit teardown; object-pool release accepts a raw identity
pointer so it never creates an aliased reference solely to return storage.
