//! # bsw-estd
//!
//! Fixed-capacity containers for embedded BSW — Rust port of OpenBSW `estd`.
//!
//! Only types that have no direct Rust core equivalent are implemented here.
//! Types like `estd::optional` → `Option`, `estd::variant` → `enum`,
//! `estd::array` → `[T; N]` are intentionally omitted.
//!
//! ## Provided types
//!
//! - [`FixedVec<T, N>`](vec::FixedVec) — fixed-capacity vector with inline storage
//! - [`ForwardList<T>`](forward_list::ForwardList) — intrusive singly-linked list
//! - [`ObjectPool<T, N>`](object_pool::ObjectPool) — fixed-capacity pool allocator
//! - [`Bitset<N, M>`](bitset::Bitset) — fixed-size bit set with `[u32; M]` storage
//! - Big-endian types: [`big_endian::BeU16`], [`big_endian::BeU32`], [`big_endian::BeU64`],
//!   [`big_endian::BeU24`], [`big_endian::BeU48`], [`big_endian::BeI16`],
//!   [`big_endian::BeI32`], [`big_endian::BeI64`]

#![cfg_attr(not(feature = "std"), no_std)]

pub mod big_endian;
pub mod bitset;
pub mod forward_list;
pub mod object_pool;
pub mod ordered_map;
pub mod vec;

pub use bitset::Bitset;
pub use forward_list::ForwardList;
pub use object_pool::ObjectPool;
pub use ordered_map::OrderedMap;
pub use vec::FixedVec;
