//! A no-alloc, sorted, vector-backed map.
//!
//! [`OrderedMap<K, V, N>`] is a Rust port of `estd::ordered_map` from
//! OpenBSW.  It stores at most `N` key-value pairs sorted by key at all
//! times, using a [`FixedVec`] as its backing store.  All operations are
//! `no_std`-compatible and perform zero heap allocation.
//!
//! # Complexity
//!
//! | Operation          | Time       |
//! |--------------------|-----------|
//! | `get` / `contains` | O(log N)  |
//! | `insert` / `remove`| O(N)      |
//! | `iter`             | O(N)      |
//!
//! # Example
//!
//! ```
//! use bsw_estd::ordered_map::OrderedMap;
//!
//! let mut map: OrderedMap<u8, &str, 8> = OrderedMap::new();
//! map.insert(3, "three").unwrap();
//! map.insert(1, "one").unwrap();
//! map.insert(2, "two").unwrap();
//!
//! // Iteration is always in sorted key order.
//! let keys: [u8; 3] = [1, 2, 3];
//! for (i, (k, _v)) in map.iter().enumerate() {
//!     assert_eq!(*k, keys[i]);
//! }
//! ```

use core::fmt;
use core::ops::Index;

use crate::vec::FixedVec;

// ---------------------------------------------------------------------------
// Primary type
// ---------------------------------------------------------------------------

/// A fixed-capacity sorted map backed by a [`FixedVec`].
///
/// Keys must implement [`Ord`].  Pairs are kept in ascending key order at
/// all times.  The capacity `N` is a const-generic parameter fixed at
/// compile time; no heap allocation ever occurs.
///
/// # Invariants
///
/// At all times:
/// - `data.len() <= N`
/// - `data[i].0 < data[i+1].0` for all valid `i` (strict ascending key order)
pub struct OrderedMap<K, V, const N: usize> {
    data: FixedVec<(K, V), N>,
}

// ---------------------------------------------------------------------------
// Constructors
// ---------------------------------------------------------------------------

impl<K: Ord, V, const N: usize> OrderedMap<K, V, N> {
    /// Creates an empty `OrderedMap`.
    #[inline]
    pub const fn new() -> Self {
        Self {
            data: FixedVec::new(),
        }
    }
}

// ---------------------------------------------------------------------------
// Default
// ---------------------------------------------------------------------------

impl<K: Ord, V, const N: usize> Default for OrderedMap<K, V, N> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Capacity
// ---------------------------------------------------------------------------

impl<K: Ord, V, const N: usize> OrderedMap<K, V, N> {
    /// Returns the number of key-value pairs currently stored.
    #[inline]
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Returns `true` when no pairs are stored.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// Returns `true` when `len == N` (capacity exhausted).
    #[inline]
    pub fn is_full(&self) -> bool {
        self.data.is_full()
    }

    /// Returns the fixed capacity `N`.
    #[inline]
    pub fn capacity(&self) -> usize {
        N
    }
}

// ---------------------------------------------------------------------------
// Lookup
// ---------------------------------------------------------------------------

impl<K: Ord, V, const N: usize> OrderedMap<K, V, N> {
    /// Returns a shared reference to the value associated with `key`, or
    /// `None` if not present.  Uses binary search — O(log N).
    pub fn get(&self, key: &K) -> Option<&V> {
        match self.data.binary_search_by(|(k, _)| k.cmp(key)) {
            Ok(idx) => Some(&self.data[idx].1),
            Err(_) => None,
        }
    }

    /// Returns a mutable reference to the value associated with `key`, or
    /// `None` if not present.  Uses binary search — O(log N).
    pub fn get_mut(&mut self, key: &K) -> Option<&mut V> {
        match self.data.binary_search_by(|(k, _)| k.cmp(key)) {
            Ok(idx) => Some(&mut self.data.as_mut_slice()[idx].1),
            Err(_) => None,
        }
    }

    /// Returns `true` if the map contains `key`.
    pub fn contains_key(&self, key: &K) -> bool {
        self.data.binary_search_by(|(k, _)| k.cmp(key)).is_ok()
    }

    /// Returns the index of the first element whose key is **greater than or
    /// equal to** `key` (i.e., the insertion point for `key`).
    pub fn lower_bound(&self, key: &K) -> usize {
        match self.data.binary_search_by(|(k, _)| k.cmp(key)) {
            Ok(idx) | Err(idx) => idx,
        }
    }

    /// Returns the index of the first element whose key is **strictly
    /// greater than** `key`.
    pub fn upper_bound(&self, key: &K) -> usize {
        // Start from lower_bound and advance past any equal key (there is at
        // most one, since keys are unique).
        let lb = self.lower_bound(key);
        if lb < self.data.len() && self.data[lb].0 == *key {
            lb + 1
        } else {
            lb
        }
    }
}

// ---------------------------------------------------------------------------
// Modification
// ---------------------------------------------------------------------------

impl<K: Ord, V, const N: usize> OrderedMap<K, V, N> {
    /// Inserts a key-value pair.
    ///
    /// - If `key` is already present, returns `Ok` with a mutable reference
    ///   to the **existing** value (no overwrite — matches C++ behaviour).
    /// - If the map is full and the key is not present, returns `Err((key,
    ///   value))` with the pair back to the caller.
    /// - Otherwise inserts the pair in sorted position and returns `Ok` with
    ///   a mutable reference to the newly inserted value.
    pub fn insert(&mut self, key: K, value: V) -> Result<&mut V, (K, V)> {
        match self.data.binary_search_by(|(k, _)| k.cmp(&key)) {
            Ok(idx) => {
                // Key already present — return reference to existing value.
                Ok(&mut self.data.as_mut_slice()[idx].1)
            }
            Err(idx) => {
                if self.data.is_full() {
                    return Err((key, value));
                }
                // insert() returns Err only when full or out-of-range;
                // we checked is_full() and idx <= len, so this is safe.
                self.data.insert(idx, (key, value)).ok();
                Ok(&mut self.data.as_mut_slice()[idx].1)
            }
        }
    }

    /// Inserts a key-value pair, **overwriting** any existing value for the
    /// same key.
    ///
    /// Returns `Ok` with a mutable reference to the (new or updated) value,
    /// or `Err((key, value))` if the map is full and the key is not already
    /// present.
    pub fn insert_or_assign(&mut self, key: K, value: V) -> Result<&mut V, (K, V)> {
        match self.data.binary_search_by(|(k, _)| k.cmp(&key)) {
            Ok(idx) => {
                // Overwrite existing value.
                self.data.as_mut_slice()[idx].1 = value;
                Ok(&mut self.data.as_mut_slice()[idx].1)
            }
            Err(idx) => {
                if self.data.is_full() {
                    return Err((key, value));
                }
                self.data.insert(idx, (key, value)).ok();
                Ok(&mut self.data.as_mut_slice()[idx].1)
            }
        }
    }

    /// Removes the entry for `key` and returns its value, or `None` if not
    /// present.
    pub fn remove(&mut self, key: &K) -> Option<V> {
        match self.data.binary_search_by(|(k, _)| k.cmp(key)) {
            Ok(idx) => Some(self.data.remove(idx).1),
            Err(_) => None,
        }
    }

    /// Removes all entries, running their destructors.
    pub fn clear(&mut self) {
        self.data.clear();
    }
}

// ---------------------------------------------------------------------------
// Entry API
// ---------------------------------------------------------------------------

/// A view into a single entry in the map, which may or may not exist.
///
/// Obtained via [`OrderedMap::entry`].
pub enum Entry<'a, K, V, const N: usize> {
    /// The entry is occupied — the key is already present.
    Occupied(OccupiedEntry<'a, K, V, N>),
    /// The entry is vacant — the key is not yet in the map.
    Vacant(VacantEntry<'a, K, V, N>),
}

/// A view into an occupied entry in the map.
pub struct OccupiedEntry<'a, K, V, const N: usize> {
    map: &'a mut OrderedMap<K, V, N>,
    idx: usize,
}

/// A view into a vacant entry in the map.
pub struct VacantEntry<'a, K, V, const N: usize> {
    map: &'a mut OrderedMap<K, V, N>,
    key: K,
    /// Sorted insertion index.
    idx: usize,
}

impl<'a, K: Ord, V, const N: usize> OccupiedEntry<'a, K, V, N> {
    /// Returns a mutable reference to the value in the entry.
    pub fn get_mut(&mut self) -> &mut V {
        &mut self.map.data.as_mut_slice()[self.idx].1
    }

    /// Converts the entry into a mutable reference to the value.
    pub fn into_mut(self) -> &'a mut V {
        &mut self.map.data.as_mut_slice()[self.idx].1
    }
}

impl<'a, K: Ord, V, const N: usize> VacantEntry<'a, K, V, N> {
    /// Inserts `value` into the map at this entry's key, returning a mutable
    /// reference to the inserted value, or `None` if the map is full.
    pub fn insert(self, value: V) -> Option<&'a mut V> {
        if self.map.data.is_full() {
            return None;
        }
        let idx = self.idx;
        self.map.data.insert(idx, (self.key, value)).ok();
        Some(&mut self.map.data.as_mut_slice()[idx].1)
    }
}

impl<'a, K: Ord, V, const N: usize> Entry<'a, K, V, N> {
    /// If occupied, yields a mutable reference to the existing value.
    /// If vacant, inserts `default` and returns a reference to the inserted
    /// value, or `None` if the map was full.
    pub fn or_insert(self, default: V) -> Option<&'a mut V> {
        match self {
            Self::Occupied(e) => Some(e.into_mut()),
            Self::Vacant(e) => e.insert(default),
        }
    }
}

impl<K: Ord, V, const N: usize> OrderedMap<K, V, N> {
    /// Returns an [`Entry`] for the given key, allowing in-place manipulation.
    pub fn entry(&mut self, key: K) -> Entry<'_, K, V, N> {
        match self.data.binary_search_by(|(k, _)| k.cmp(&key)) {
            Ok(idx) => Entry::Occupied(OccupiedEntry { map: self, idx }),
            Err(idx) => Entry::Vacant(VacantEntry {
                map: self,
                key,
                idx,
            }),
        }
    }
}

// ---------------------------------------------------------------------------
// Iteration types
// ---------------------------------------------------------------------------

/// An iterator over `(&K, &V)` pairs in sorted key order.
pub struct Iter<'a, K, V> {
    inner: core::slice::Iter<'a, (K, V)>,
}

impl<'a, K, V> Iterator for Iter<'a, K, V> {
    type Item = (&'a K, &'a V);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next().map(|(k, v)| (k, v))
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        self.inner.size_hint()
    }
}

impl<K, V> ExactSizeIterator for Iter<'_, K, V> {}

/// An iterator over `(&K, &mut V)` pairs in sorted key order.
pub struct IterMut<'a, K, V> {
    inner: core::slice::IterMut<'a, (K, V)>,
}

impl<'a, K, V> Iterator for IterMut<'a, K, V> {
    type Item = (&'a K, &'a mut V);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next().map(|(k, v)| (&*k, v))
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        self.inner.size_hint()
    }
}

impl<K, V> ExactSizeIterator for IterMut<'_, K, V> {}

/// An iterator over `&K` in sorted order.
pub struct Keys<'a, K, V> {
    inner: core::slice::Iter<'a, (K, V)>,
}

impl<'a, K, V> Iterator for Keys<'a, K, V> {
    type Item = &'a K;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next().map(|(k, _)| k)
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        self.inner.size_hint()
    }
}

impl<K, V> ExactSizeIterator for Keys<'_, K, V> {}

/// An iterator over `&V` in sorted key order.
pub struct Values<'a, K, V> {
    inner: core::slice::Iter<'a, (K, V)>,
}

impl<'a, K, V> Iterator for Values<'a, K, V> {
    type Item = &'a V;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next().map(|(_, v)| v)
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        self.inner.size_hint()
    }
}

impl<K, V> ExactSizeIterator for Values<'_, K, V> {}

// ---------------------------------------------------------------------------
// Iteration methods
// ---------------------------------------------------------------------------

impl<K: Ord, V, const N: usize> OrderedMap<K, V, N> {
    /// Returns an iterator over `(&K, &V)` pairs in sorted key order.
    pub fn iter(&self) -> Iter<'_, K, V> {
        Iter {
            inner: self.data.as_slice().iter(),
        }
    }

    /// Returns an iterator over `(&K, &mut V)` pairs in sorted key order.
    pub fn iter_mut(&mut self) -> IterMut<'_, K, V> {
        IterMut {
            inner: self.data.as_mut_slice().iter_mut(),
        }
    }

    /// Returns an iterator over `&K` in sorted order.
    pub fn keys(&self) -> Keys<'_, K, V> {
        Keys {
            inner: self.data.as_slice().iter(),
        }
    }

    /// Returns an iterator over `&V` in sorted key order.
    pub fn values(&self) -> Values<'_, K, V> {
        Values {
            inner: self.data.as_slice().iter(),
        }
    }
}

// ---------------------------------------------------------------------------
// IntoIterator
// ---------------------------------------------------------------------------

impl<'a, K: Ord, V, const N: usize> IntoIterator for &'a OrderedMap<K, V, N> {
    type Item = (&'a K, &'a V);
    type IntoIter = Iter<'a, K, V>;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl<'a, K: Ord, V, const N: usize> IntoIterator for &'a mut OrderedMap<K, V, N> {
    type Item = (&'a K, &'a mut V);
    type IntoIter = IterMut<'a, K, V>;

    #[inline]
    fn into_iter(self) -> Self::IntoIter {
        self.iter_mut()
    }
}

// ---------------------------------------------------------------------------
// Index<&K>  — panics if not found (matches C++ `at()`)
// ---------------------------------------------------------------------------

impl<K: Ord, V, const N: usize> Index<&K> for OrderedMap<K, V, N> {
    type Output = V;

    fn index(&self, key: &K) -> &V {
        self.get(key).expect("OrderedMap: key not found")
    }
}

// ---------------------------------------------------------------------------
// Debug
// ---------------------------------------------------------------------------

impl<K: Ord + fmt::Debug, V: fmt::Debug, const N: usize> fmt::Debug for OrderedMap<K, V, N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut dm = f.debug_map();
        for (k, v) in self.iter() {
            dm.entry(k, v);
        }
        dm.finish()
    }
}

// ---------------------------------------------------------------------------
// PartialEq / Eq
// ---------------------------------------------------------------------------

impl<K: Ord + PartialEq, V: PartialEq, const N: usize, const M: usize>
    PartialEq<OrderedMap<K, V, M>> for OrderedMap<K, V, N>
{
    fn eq(&self, other: &OrderedMap<K, V, M>) -> bool {
        self.data.as_slice() == other.data.as_slice()
    }
}

impl<K: Ord + Eq, V: Eq, const N: usize> Eq for OrderedMap<K, V, N> {}

// ---------------------------------------------------------------------------
// Clone
// ---------------------------------------------------------------------------

impl<K: Ord + Clone, V: Clone, const N: usize> Clone for OrderedMap<K, V, N> {
    fn clone(&self) -> Self {
        Self {
            data: self.data.clone(),
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::OrderedMap;

    // -----------------------------------------------------------------------
    // 1. Empty map operations
    // -----------------------------------------------------------------------

    #[test]
    fn empty_map_has_zero_len() {
        let map: OrderedMap<i32, i32, 8> = OrderedMap::new();
        assert_eq!(map.len(), 0);
        assert!(map.is_empty());
        assert!(!map.is_full());
        assert_eq!(map.capacity(), 8);
    }

    #[test]
    fn empty_map_get_returns_none() {
        let map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        assert_eq!(map.get(&1), None);
        assert!(!map.contains_key(&1));
    }

    #[test]
    fn empty_map_remove_returns_none() {
        let mut map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        assert_eq!(map.remove(&42), None);
    }

    #[test]
    fn empty_map_iter_is_empty() {
        let map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        assert_eq!(map.iter().count(), 0);
    }

    #[test]
    fn default_produces_empty_map() {
        let map: OrderedMap<i32, i32, 4> = OrderedMap::default();
        assert!(map.is_empty());
    }

    // -----------------------------------------------------------------------
    // 2. Insert single element
    // -----------------------------------------------------------------------

    #[test]
    fn insert_single_element() {
        let mut map: OrderedMap<i32, &str, 4> = OrderedMap::new();
        let result = map.insert(1, "one");
        assert!(result.is_ok());
        assert_eq!(map.len(), 1);
        assert_eq!(map.get(&1), Some(&"one"));
    }

    // -----------------------------------------------------------------------
    // 3. Insert maintains sorted order
    // -----------------------------------------------------------------------

    #[test]
    fn insert_maintains_sorted_order() {
        let mut map: OrderedMap<i32, i32, 8> = OrderedMap::new();
        map.insert(3, 30).unwrap();
        map.insert(1, 10).unwrap();
        map.insert(5, 50).unwrap();
        map.insert(2, 20).unwrap();
        map.insert(4, 40).unwrap();

        let keys: Vec<i32> = map.keys().copied().collect();
        assert_eq!(keys, [1, 2, 3, 4, 5]);
    }

    // -----------------------------------------------------------------------
    // 4. Insert duplicate key returns existing value (no overwrite)
    // -----------------------------------------------------------------------

    #[test]
    fn insert_duplicate_key_returns_existing() {
        let mut map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        map.insert(1, 100).unwrap();
        let val_ref = map.insert(1, 999).unwrap();
        // Should get back the *existing* value (100), not 999.
        assert_eq!(*val_ref, 100);
        // Map should still have only one entry.
        assert_eq!(map.len(), 1);
        assert_eq!(map.get(&1), Some(&100));
    }

    // -----------------------------------------------------------------------
    // 5. insert_or_assign overwrites existing value
    // -----------------------------------------------------------------------

    #[test]
    fn insert_or_assign_overwrites_existing() {
        let mut map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        map.insert(1, 100).unwrap();
        let val_ref = map.insert_or_assign(1, 999).unwrap();
        assert_eq!(*val_ref, 999);
        assert_eq!(map.len(), 1);
        assert_eq!(map.get(&1), Some(&999));
    }

    #[test]
    fn insert_or_assign_inserts_new_key() {
        let mut map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        map.insert_or_assign(42, 1).unwrap();
        assert_eq!(map.get(&42), Some(&1));
    }

    // -----------------------------------------------------------------------
    // 6. Remove existing key
    // -----------------------------------------------------------------------

    #[test]
    fn remove_existing_key_returns_value() {
        let mut map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        map.insert(1, 10).unwrap();
        map.insert(2, 20).unwrap();
        map.insert(3, 30).unwrap();

        assert_eq!(map.remove(&2), Some(20));
        assert_eq!(map.len(), 2);
        assert_eq!(map.get(&2), None);
        // Remaining elements still in sorted order.
        let keys: Vec<i32> = map.keys().copied().collect();
        assert_eq!(keys, [1, 3]);
    }

    // -----------------------------------------------------------------------
    // 7. Remove non-existing key returns None
    // -----------------------------------------------------------------------

    #[test]
    fn remove_nonexistent_key_returns_none() {
        let mut map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        map.insert(1, 10).unwrap();
        assert_eq!(map.remove(&99), None);
        assert_eq!(map.len(), 1);
    }

    // -----------------------------------------------------------------------
    // 8. get / get_mut / contains_key
    // -----------------------------------------------------------------------

    #[test]
    fn get_returns_correct_value() {
        let mut map: OrderedMap<&str, i32, 4> = OrderedMap::new();
        map.insert("hello", 1).unwrap();
        map.insert("world", 2).unwrap();

        assert_eq!(map.get(&"hello"), Some(&1));
        assert_eq!(map.get(&"world"), Some(&2));
        assert_eq!(map.get(&"missing"), None);
    }

    #[test]
    fn get_mut_allows_in_place_modification() {
        let mut map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        map.insert(5, 50).unwrap();

        if let Some(v) = map.get_mut(&5) {
            *v = 500;
        }
        assert_eq!(map.get(&5), Some(&500));
    }

    #[test]
    fn get_mut_returns_none_for_missing() {
        let mut map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        assert_eq!(map.get_mut(&1), None);
    }

    #[test]
    fn contains_key_returns_correct_bool() {
        let mut map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        map.insert(10, 100).unwrap();
        assert!(map.contains_key(&10));
        assert!(!map.contains_key(&99));
    }

    // -----------------------------------------------------------------------
    // 9. lower_bound / upper_bound
    // -----------------------------------------------------------------------

    #[test]
    fn lower_bound_on_existing_key() {
        let mut map: OrderedMap<i32, i32, 8> = OrderedMap::new();
        map.insert(2, 2).unwrap();
        map.insert(4, 4).unwrap();
        map.insert(6, 6).unwrap();

        // lower_bound(4) => index 1 (where key 4 lives)
        assert_eq!(map.lower_bound(&4), 1);
    }

    #[test]
    fn lower_bound_on_missing_key() {
        let mut map: OrderedMap<i32, i32, 8> = OrderedMap::new();
        map.insert(2, 2).unwrap();
        map.insert(4, 4).unwrap();
        map.insert(6, 6).unwrap();

        // lower_bound(3) => index 1 (first key >= 3 is 4 at index 1)
        assert_eq!(map.lower_bound(&3), 1);
        // lower_bound(5) => index 2 (first key >= 5 is 6 at index 2)
        assert_eq!(map.lower_bound(&5), 2);
        // lower_bound(0) => index 0 (all keys >= 0)
        assert_eq!(map.lower_bound(&0), 0);
        // lower_bound(7) => index 3 (past-the-end)
        assert_eq!(map.lower_bound(&7), 3);
    }

    #[test]
    fn upper_bound_on_existing_key() {
        let mut map: OrderedMap<i32, i32, 8> = OrderedMap::new();
        map.insert(2, 2).unwrap();
        map.insert(4, 4).unwrap();
        map.insert(6, 6).unwrap();

        // upper_bound(4) => index 2 (first key strictly > 4 is 6 at index 2)
        assert_eq!(map.upper_bound(&4), 2);
    }

    #[test]
    fn upper_bound_on_missing_key() {
        let mut map: OrderedMap<i32, i32, 8> = OrderedMap::new();
        map.insert(2, 2).unwrap();
        map.insert(4, 4).unwrap();
        map.insert(6, 6).unwrap();

        // upper_bound(3) => index 1 (first key > 3 is 4 at index 1)
        assert_eq!(map.upper_bound(&3), 1);
        // upper_bound(7) => index 3 (past-the-end)
        assert_eq!(map.upper_bound(&7), 3);
    }

    // -----------------------------------------------------------------------
    // 10. Iterator yields sorted order
    // -----------------------------------------------------------------------

    #[test]
    fn iter_yields_sorted_order() {
        let mut map: OrderedMap<i32, i32, 8> = OrderedMap::new();
        map.insert(30, 3).unwrap();
        map.insert(10, 1).unwrap();
        map.insert(20, 2).unwrap();

        let pairs: Vec<(i32, i32)> = map.iter().map(|(&k, &v)| (k, v)).collect();
        assert_eq!(pairs, [(10, 1), (20, 2), (30, 3)]);
    }

    #[test]
    fn into_iter_ref_yields_sorted_order() {
        let mut map: OrderedMap<i32, i32, 8> = OrderedMap::new();
        map.insert(5, 50).unwrap();
        map.insert(3, 30).unwrap();
        map.insert(7, 70).unwrap();

        let mut keys = Vec::new();
        for (k, _v) in &map {
            keys.push(*k);
        }
        assert_eq!(keys, [3, 5, 7]);
    }

    // -----------------------------------------------------------------------
    // 11. iter_mut allows modification
    // -----------------------------------------------------------------------

    #[test]
    fn iter_mut_allows_value_modification() {
        let mut map: OrderedMap<i32, i32, 8> = OrderedMap::new();
        map.insert(1, 10).unwrap();
        map.insert(2, 20).unwrap();
        map.insert(3, 30).unwrap();

        for (_k, v) in map.iter_mut() {
            *v *= 10;
        }

        assert_eq!(map.get(&1), Some(&100));
        assert_eq!(map.get(&2), Some(&200));
        assert_eq!(map.get(&3), Some(&300));
    }

    #[test]
    fn into_iter_mut_allows_value_modification() {
        let mut map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        map.insert(1, 1).unwrap();
        map.insert(2, 2).unwrap();

        for (_k, v) in &mut map {
            *v += 100;
        }

        assert_eq!(map.get(&1), Some(&101));
        assert_eq!(map.get(&2), Some(&102));
    }

    // -----------------------------------------------------------------------
    // 12. Full map returns Err on insert
    // -----------------------------------------------------------------------

    #[test]
    fn insert_returns_err_when_full() {
        let mut map: OrderedMap<i32, i32, 3> = OrderedMap::new();
        map.insert(1, 10).unwrap();
        map.insert(2, 20).unwrap();
        map.insert(3, 30).unwrap();
        assert!(map.is_full());

        // A new key should be rejected.
        let result = map.insert(4, 40);
        assert!(result.is_err());
        let (k, v) = result.unwrap_err();
        assert_eq!(k, 4);
        assert_eq!(v, 40);
        // Map unchanged.
        assert_eq!(map.len(), 3);
    }

    #[test]
    fn insert_or_assign_returns_err_when_full_and_new_key() {
        let mut map: OrderedMap<i32, i32, 2> = OrderedMap::new();
        map.insert(1, 10).unwrap();
        map.insert(2, 20).unwrap();

        let result = map.insert_or_assign(3, 30);
        assert!(result.is_err());
    }

    // -----------------------------------------------------------------------
    // 13. Clear resets to empty
    // -----------------------------------------------------------------------

    #[test]
    fn clear_resets_to_empty() {
        let mut map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        map.insert(1, 10).unwrap();
        map.insert(2, 20).unwrap();
        map.clear();
        assert!(map.is_empty());
        assert_eq!(map.len(), 0);
        assert_eq!(map.get(&1), None);
    }

    // -----------------------------------------------------------------------
    // 14. Index trait panics on missing key
    // -----------------------------------------------------------------------

    #[test]
    fn index_returns_value_for_existing_key() {
        let mut map: OrderedMap<i32, &str, 4> = OrderedMap::new();
        map.insert(1, "one").unwrap();
        assert_eq!(map[&1], "one");
    }

    #[test]
    #[should_panic(expected = "key not found")]
    fn index_panics_on_missing_key() {
        let map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        let _ = map[&99];
    }

    // -----------------------------------------------------------------------
    // 15. PartialEq between maps
    // -----------------------------------------------------------------------

    #[test]
    fn partial_eq_same_contents() {
        let mut a: OrderedMap<i32, i32, 4> = OrderedMap::new();
        let mut b: OrderedMap<i32, i32, 4> = OrderedMap::new();
        a.insert(1, 10).unwrap();
        a.insert(2, 20).unwrap();
        b.insert(2, 20).unwrap();
        b.insert(1, 10).unwrap(); // inserted in different order

        assert_eq!(a, b);
    }

    #[test]
    fn partial_eq_different_contents() {
        let mut a: OrderedMap<i32, i32, 4> = OrderedMap::new();
        let mut b: OrderedMap<i32, i32, 4> = OrderedMap::new();
        a.insert(1, 10).unwrap();
        b.insert(1, 99).unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn partial_eq_different_sizes() {
        let mut a: OrderedMap<i32, i32, 4> = OrderedMap::new();
        let mut b: OrderedMap<i32, i32, 8> = OrderedMap::new();
        a.insert(1, 10).unwrap();
        b.insert(1, 10).unwrap();
        b.insert(2, 20).unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn empty_maps_are_equal() {
        let a: OrderedMap<i32, i32, 4> = OrderedMap::new();
        let b: OrderedMap<i32, i32, 4> = OrderedMap::new();
        assert_eq!(a, b);
    }

    // -----------------------------------------------------------------------
    // 16. Clone produces independent copy
    // -----------------------------------------------------------------------

    #[test]
    fn clone_produces_independent_copy() {
        let mut original: OrderedMap<i32, i32, 8> = OrderedMap::new();
        original.insert(1, 10).unwrap();
        original.insert(2, 20).unwrap();

        let mut cloned = original.clone();
        assert_eq!(original, cloned);

        // Mutating clone does not affect original.
        cloned.insert(3, 30).unwrap();
        assert_eq!(cloned.len(), 3);
        assert_eq!(original.len(), 2);
        assert_eq!(original.get(&3), None);
    }

    #[test]
    fn clone_of_empty_map_is_empty() {
        let map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        let cloned = map.clone();
        assert!(cloned.is_empty());
    }

    // -----------------------------------------------------------------------
    // 17. Zero-capacity map behaviour
    // -----------------------------------------------------------------------

    #[test]
    fn zero_capacity_is_full_and_empty() {
        let map: OrderedMap<i32, i32, 0> = OrderedMap::new();
        assert!(map.is_empty());
        assert!(map.is_full());
        assert_eq!(map.len(), 0);
        assert_eq!(map.capacity(), 0);
    }

    #[test]
    fn zero_capacity_insert_returns_err() {
        let mut map: OrderedMap<i32, i32, 0> = OrderedMap::new();
        let result = map.insert(1, 10);
        assert!(result.is_err());
        let (k, v) = result.unwrap_err();
        assert_eq!(k, 1);
        assert_eq!(v, 10);
    }

    #[test]
    fn zero_capacity_contains_key_is_false() {
        let map: OrderedMap<i32, i32, 0> = OrderedMap::new();
        assert!(!map.contains_key(&1));
    }

    // -----------------------------------------------------------------------
    // 18. keys() / values() iterators
    // -----------------------------------------------------------------------

    #[test]
    fn keys_yields_sorted_keys() {
        let mut map: OrderedMap<i32, i32, 8> = OrderedMap::new();
        map.insert(30, 3).unwrap();
        map.insert(10, 1).unwrap();
        map.insert(20, 2).unwrap();

        let keys: Vec<i32> = map.keys().copied().collect();
        assert_eq!(keys, [10, 20, 30]);
    }

    #[test]
    fn values_yields_values_in_key_order() {
        let mut map: OrderedMap<i32, &str, 8> = OrderedMap::new();
        map.insert(3, "three").unwrap();
        map.insert(1, "one").unwrap();
        map.insert(2, "two").unwrap();

        let vals: Vec<&&str> = map.values().collect();
        assert_eq!(vals, [&"one", &"two", &"three"]);
    }

    #[test]
    fn keys_empty_map_is_empty_iterator() {
        let map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        assert_eq!(map.keys().count(), 0);
    }

    #[test]
    fn values_empty_map_is_empty_iterator() {
        let map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        assert_eq!(map.values().count(), 0);
    }

    // -----------------------------------------------------------------------
    // Additional: exact_size_iterator
    // -----------------------------------------------------------------------

    #[test]
    fn iter_exact_size() {
        let mut map: OrderedMap<i32, i32, 8> = OrderedMap::new();
        map.insert(1, 1).unwrap();
        map.insert(2, 2).unwrap();
        map.insert(3, 3).unwrap();

        let mut it = map.iter();
        assert_eq!(it.len(), 3);
        let _ = it.next();
        assert_eq!(it.len(), 2);
    }

    // -----------------------------------------------------------------------
    // Additional: Debug formatting smoke test
    // -----------------------------------------------------------------------

    #[test]
    fn debug_does_not_panic() {
        let mut map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        map.insert(1, 10).unwrap();
        map.insert(2, 20).unwrap();
        let _s = ::core::format_args!("{:?}", map);
    }

    // -----------------------------------------------------------------------
    // Additional: entry API
    // -----------------------------------------------------------------------

    #[test]
    fn entry_or_insert_on_vacant() {
        let mut map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        let val = map.entry(1).or_insert(42);
        assert_eq!(*val.unwrap(), 42);
        assert_eq!(map.get(&1), Some(&42));
    }

    #[test]
    fn entry_or_insert_on_occupied_does_not_overwrite() {
        let mut map: OrderedMap<i32, i32, 4> = OrderedMap::new();
        map.insert(1, 100).unwrap();
        let val = map.entry(1).or_insert(999);
        // Should keep existing value 100.
        assert_eq!(*val.unwrap(), 100);
        assert_eq!(map.get(&1), Some(&100));
    }

    #[test]
    fn entry_vacant_on_full_map_returns_none() {
        let mut map: OrderedMap<i32, i32, 1> = OrderedMap::new();
        map.insert(1, 10).unwrap();
        // Map is full; inserting via vacant entry should return None.
        let result = map.entry(2).or_insert(20);
        assert!(result.is_none());
    }

    // -----------------------------------------------------------------------
    // Additional: remove shifts remaining elements
    // -----------------------------------------------------------------------

    #[test]
    fn remove_first_element_shifts_correctly() {
        let mut map: OrderedMap<i32, i32, 8> = OrderedMap::new();
        for i in 1..=5 {
            map.insert(i, i * 10).unwrap();
        }
        map.remove(&1);
        let keys: Vec<i32> = map.keys().copied().collect();
        assert_eq!(keys, [2, 3, 4, 5]);
    }

    #[test]
    fn remove_last_element() {
        let mut map: OrderedMap<i32, i32, 8> = OrderedMap::new();
        for i in 1..=5 {
            map.insert(i, i * 10).unwrap();
        }
        let v = map.remove(&5);
        assert_eq!(v, Some(50));
        let keys: Vec<i32> = map.keys().copied().collect();
        assert_eq!(keys, [1, 2, 3, 4]);
    }
}
