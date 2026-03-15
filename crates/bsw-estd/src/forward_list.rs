// Copyright 2024 Accenture.
// SPDX-License-Identifier: Apache-2.0

//! Intrusive singly-linked forward list — Rust port of OpenBSW `estd::forward_list`.
//!
//! # Design
//!
//! This is a **non-owning**, **sentinel-based**, **intrusive** singly-linked list.
//! Nodes embed a [`Link<T>`] field and are **externally owned** — the list never
//! allocates or frees memory.
//!
//! ## Sentinel convention (mirrors C++)
//!
//! The C++ uses `reinterpret_cast<node*>(1)` as a "not in any list" sentinel.
//! Here we use [`LinkState::Free`] for the same purpose.
//!
//! | State              | C++ `_next` value    | Rust `LinkState`                  |
//! |--------------------|----------------------|-----------------------------------|
//! | Not in any list    | `(node*)1`           | `Free`                            |
//! | End of list        | `nullptr`            | `InList(None)`                    |
//! | Has a successor    | valid pointer        | `InList(Some(ptr))`               |
//!
//! The list head is a **sentinel `Link<T>`** embedded in `ForwardList` itself.
//! Its `next` always points to the first real node (or `None` when empty), and
//! it is permanently in `InList` state (never `Free`).
//!
//! ## Thread safety
//!
//! `ForwardList` is `!Send` and `!Sync` because it contains raw pointers and uses
//! `UnsafeCell` for interior mutability.  Single-threaded / interrupt-safe
//! embedded use is the intended target.
//!
//! # Usage
//!
//! ```rust,ignore
//! use bsw_estd::forward_list::{ForwardList, Link};
//! use bsw_estd::impl_linked;
//!
//! struct MyNode {
//!     value: u32,
//!     link: Link<MyNode>,
//! }
//! impl MyNode {
//!     fn new(v: u32) -> Self { MyNode { value: v, link: Link::new() } }
//! }
//! impl_linked!(MyNode, link);
//!
//! let mut list: ForwardList<MyNode> = ForwardList::new();
//! let n1 = MyNode::new(1);
//! let n2 = MyNode::new(2);
//! list.push_front(&n1);
//! list.push_front(&n2);
//! assert_eq!(list.len(), 2);
//! ```

use core::cell::UnsafeCell;
use core::marker::PhantomData;
use core::ptr::NonNull;

// ──────────────────────────────────────────────────────────────────────────────
// LinkState
// ──────────────────────────────────────────────────────────────────────────────

/// Internal state of a [`Link`] node.
///
/// Stored inside an [`UnsafeCell`] so that the list can mutate link pointers
/// through shared references (interior mutability), matching the C++ design
/// where const methods manipulate raw mutable pointers.
#[derive(Debug)]
enum LinkState<T> {
    /// Node is not part of any list.  Equivalent to C++ `_next == (node*)1`.
    Free,
    /// Node is part of a list.
    ///
    /// - `None`       → this is the last node (C++ `_next == nullptr`)
    /// - `Some(ptr)`  → pointer to the next [`Link`] in the list
    InList(Option<NonNull<Link<T>>>),
}

// ──────────────────────────────────────────────────────────────────────────────
// Link<T>
// ──────────────────────────────────────────────────────────────────────────────

/// Intrusive link node.  Embed one of these in every struct you want to store
/// in a [`ForwardList`].
///
/// A single `Link<T>` can only be in **one** `ForwardList<T>` at a time.
/// If you need a type to participate in two independent lists simultaneously,
/// embed two link fields with different type parameters (see C++ CRTP example
/// in the original `estd::forward_list` documentation).
///
/// # Example
///
/// ```rust,ignore
/// use bsw_estd::forward_list::Link;
///
/// struct Node {
///     data: u32,
///     link: Link<Node>,
/// }
/// ```
pub struct Link<T> {
    next: UnsafeCell<LinkState<T>>,
}

impl<T> Default for Link<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> Link<T> {
    /// Creates a new link in the [`LinkState::Free`] state (not in any list).
    ///
    /// This is a `const fn` so it can be used in `const`/`static` contexts.
    #[inline]
    pub const fn new() -> Self {
        Link {
            next: UnsafeCell::new(LinkState::Free),
        }
    }

    /// Returns `true` if this link is currently part of a list.
    ///
    /// Equivalent to `estd::is_in_use(node)` in C++.
    #[inline]
    pub fn is_in_use(&self) -> bool {
        // SAFETY: We read the UnsafeCell only through a shared reference.
        // No concurrent mutation is possible: ForwardList is !Sync and all
        // list operations take &mut ForwardList, preventing aliased mutation.
        match unsafe { &*self.next.get() } {
            LinkState::Free => false,
            LinkState::InList(_) => true,
        }
    }

    // ── Internal helpers (visible only within this module) ────────────────────

    /// Reads the next pointer without checking the Free state.
    ///
    /// # Safety
    /// Caller must ensure no concurrent mutation of this link's `next` field
    /// and that this link is currently in `InList` state (or is the sentinel).
    #[inline]
    unsafe fn get_next(&self) -> Option<NonNull<Link<T>>> {
        match &*self.next.get() {
            LinkState::Free => None,
            LinkState::InList(p) => *p,
        }
    }

    /// Writes the next pointer, transitioning to `InList` state.
    ///
    /// # Safety
    /// Caller must ensure exclusive access to this link's `next` field and
    /// that `ptr` (if `Some`) points to a valid, live `Link<T>`.
    #[inline]
    unsafe fn set_next(&self, ptr: Option<NonNull<Link<T>>>) {
        *self.next.get() = LinkState::InList(ptr);
    }

    /// Marks this link as free (not in any list).
    ///
    /// # Safety
    /// Caller must ensure exclusive access and that this link has been
    /// unlinked from every list before calling.
    #[inline]
    unsafe fn set_free(&self) {
        *self.next.get() = LinkState::Free;
    }
}

impl<T> core::fmt::Debug for Link<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // SAFETY: Read-only access during formatting; no concurrent mutation.
        let state = unsafe { &*self.next.get() };
        match state {
            LinkState::Free => write!(f, "Link(Free)"),
            LinkState::InList(None) => write!(f, "Link(InList(end))"),
            LinkState::InList(Some(p)) => write!(f, "Link(InList({:p}))", p.as_ptr()),
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Linked trait
// ──────────────────────────────────────────────────────────────────────────────

/// Trait implemented by types that embed a [`Link<Self>`] field.
///
/// # Safety
///
/// Implementors **must** uphold **both** of the following invariants:
///
/// 1. `link()` returns a reference to a `Link<Self>` field at a **fixed,
///    stable byte offset** within `Self`.  The offset must not change across
///    calls or across the lifetime of any instance.
/// 2. `from_link_ptr(ptr)` correctly computes the address of the containing
///    `Self` instance given a pointer to that same `Link<Self>` field.
///    Passing the wrong field name to [`impl_linked!`] causes undefined
///    behaviour.
///
/// Use the [`impl_linked!`] macro which generates a correct implementation
/// using `core::mem::offset_of!`.
pub unsafe trait Linked: Sized {
    /// Returns a shared reference to the embedded [`Link`] field.
    fn link(&self) -> &Link<Self>;

    /// Converts a pointer to the [`Link`] field back to a pointer to `Self`.
    ///
    /// # Safety
    ///
    /// `ptr` must point to the `Link<Self>` field **inside a valid, live `Self`
    /// instance**.  The pointer must be correctly aligned and the instance must
    /// not have been dropped.
    unsafe fn from_link_ptr(ptr: *const Link<Self>) -> *const Self;
}

// ──────────────────────────────────────────────────────────────────────────────
// impl_linked! macro
// ──────────────────────────────────────────────────────────────────────────────

/// Implements [`Linked`] for a type that has a named [`Link`] field.
///
/// # Usage
///
/// ```rust,ignore
/// use bsw_estd::forward_list::Link;
/// use bsw_estd::impl_linked;
///
/// struct MyNode {
///     value: u32,
///     link: Link<MyNode>,
/// }
/// impl_linked!(MyNode, link);
/// ```
///
/// This expands to `unsafe impl Linked for MyNode` using
/// `core::mem::offset_of!(MyNode, link)` (stable since Rust 1.77) to compute
/// the container address from a link pointer.
#[macro_export]
macro_rules! impl_linked {
    ($type:ty, $field:ident) => {
        unsafe impl $crate::forward_list::Linked for $type {
            #[inline]
            fn link(&self) -> &$crate::forward_list::Link<Self> {
                &self.$field
            }

            #[inline]
            unsafe fn from_link_ptr(ptr: *const $crate::forward_list::Link<Self>) -> *const Self {
                // SAFETY: Caller guarantees `ptr` points to the `$field` member
                // of a valid, live `$type` instance.  We subtract the known
                // compile-time offset of that field to recover the container address.
                let offset = core::mem::offset_of!($type, $field);
                (ptr as *const u8).sub(offset) as *const Self
            }
        }
    };
}

// ──────────────────────────────────────────────────────────────────────────────
// ForwardList<T>
// ──────────────────────────────────────────────────────────────────────────────

/// Non-owning intrusive singly-linked forward list.
///
/// Nodes must embed a [`Link<T>`] field and implement the [`Linked`] trait
/// (use [`impl_linked!`] for the boilerplate).  The list **does not own** the
/// nodes — they are owned by the caller and must outlive any list they are
/// inserted into.
///
/// ## Complexity
///
/// | Operation    | Complexity |
/// |--------------|------------|
/// | `push_front` | O(1)       |
/// | `pop_front`  | O(1)       |
/// | `front`      | O(1)       |
/// | `is_empty`   | O(1)       |
/// | `len`        | O(n)       |
/// | `remove`     | O(n)       |
/// | `contains`   | O(n)       |
/// | `reverse`    | O(n)       |
/// | `clear`      | O(n)       |
///
/// ## Drop behaviour
///
/// Dropping a `ForwardList` marks all remaining nodes as [`LinkState::Free`]
/// so they can safely be inserted into another list afterwards — mirroring the
/// C++ destructor which calls `clear()`.
pub struct ForwardList<T: Linked> {
    /// Sentinel "before-first" link.  Its `next` field points to the first
    /// real node, or is `None` when the list is empty.  The sentinel is always
    /// in `InList` state — it is never `Free`.
    ///
    /// Note: `const fn new()` constructs this in `Free` state (the only
    /// const-constructable state).  [`ensure_head_init`] promotes it to
    /// `InList(None)` on the first mutation.  All read helpers treat `Free`
    /// on the sentinel as equivalent to `InList(None)`.
    head: Link<T>,
    // ForwardList contains raw pointers — it is !Send and !Sync by default
    // because PhantomData<*mut T> opts out of both auto-traits.
    _not_send_sync: PhantomData<*mut T>,
}

impl<T: Linked> ForwardList<T> {
    /// Creates an empty list.
    ///
    /// This is a `const fn` and can be used to initialise `static` lists.
    #[inline]
    pub const fn new() -> Self {
        ForwardList {
            head: Link::new(),
            _not_send_sync: PhantomData,
        }
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /// Ensures the sentinel head link is in `InList(None)` state.
    ///
    /// `const fn new()` constructs the sentinel in `Free` state.  We lazily
    /// promote it to `InList(None)` on the first mutation so that all internal
    /// helpers that use `set_next`/`get_next` can work uniformly.
    ///
    /// # Safety
    /// Must be called while holding `&mut ForwardList` (exclusive access).
    #[inline]
    unsafe fn ensure_head_init(&self) {
        // SAFETY: Exclusive access is guaranteed by the &mut ForwardList borrow
        // held by the caller.  Reading and writing through UnsafeCell is safe
        // when there is no concurrent access.
        if matches!(&*self.head.next.get(), LinkState::Free) {
            *self.head.next.get() = LinkState::InList(None);
        }
    }

    /// Returns the next pointer stored in the sentinel head.
    ///
    /// Treats `Free` (uninitialised sentinel) as `None` (empty list).
    ///
    /// # Safety
    /// Caller must hold `&mut ForwardList` or ensure no concurrent mutation.
    #[inline]
    unsafe fn head_next(&self) -> Option<NonNull<Link<T>>> {
        // SAFETY: see ensure_head_init.
        match &*self.head.next.get() {
            LinkState::Free => None,
            LinkState::InList(p) => *p,
        }
    }

    /// Writes the sentinel's next pointer directly.
    ///
    /// # Safety
    /// Caller must hold exclusive access (`&mut ForwardList`).  `ptr` (if
    /// `Some`) must point to a valid, live `Link<T>` that is already in
    /// `InList` state.
    #[inline]
    unsafe fn set_head_next(&self, ptr: Option<NonNull<Link<T>>>) {
        // SAFETY: see ensure_head_init.
        *self.head.next.get() = LinkState::InList(ptr);
    }

    /// Converts a `NonNull<Link<T>>` to `&'a T`.
    ///
    /// # Safety
    /// `link_ptr` must point to the `Link<T>` field inside a valid, live `T`
    /// whose lifetime extends for at least `'a`.
    #[inline]
    unsafe fn link_ptr_to_ref<'a>(link_ptr: NonNull<Link<T>>) -> &'a T {
        // SAFETY: Caller guarantees link_ptr is inside a valid T.  We use the
        // implementor-supplied from_link_ptr to recover the container address,
        // then form a shared reference bound to 'a.
        let node_ptr = T::from_link_ptr(link_ptr.as_ptr());
        &*node_ptr
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /// Returns `true` if the list contains no elements.
    #[inline]
    pub fn is_empty(&self) -> bool {
        // SAFETY: head_next is read-only; no concurrent mutation (ForwardList is !Sync).
        unsafe { self.head_next().is_none() }
    }

    /// Returns the number of elements in the list.
    ///
    /// This is **O(n)** — it walks the entire list.
    pub fn len(&self) -> usize {
        self.iter().count()
    }

    /// Returns a shared reference to the first element, or `None` if the list
    /// is empty.
    pub fn front(&self) -> Option<&T> {
        // SAFETY: head_next reads the sentinel.  link_ptr_to_ref is safe
        // because nodes in the list are guaranteed live by the caller's
        // ownership contract (nodes must outlive the list).
        unsafe {
            self.head_next()
                .map(|link_ptr| Self::link_ptr_to_ref(link_ptr))
        }
    }

    /// Inserts `node` at the front of the list.
    ///
    /// If `node` is already part of **any** list (`node.link().is_in_use()`
    /// returns `true`), this is a **no-op** — matching C++ behaviour where
    /// `estd::forward_list::push_front` guards with `!is_in_use(value)`.
    pub fn push_front(&mut self, node: &T) {
        let link = node.link();
        if link.is_in_use() {
            // No-op: already in a list.  Matches C++ estd::forward_list::push_front.
            return;
        }
        // SAFETY: We hold &mut self so no other access to the list or its
        // sentinel is possible.  `link` is not in use (checked above), so it
        // is safe to write to its next field.  NonNull::new_unchecked is safe
        // because `link` comes from a Rust reference and is therefore non-null.
        unsafe {
            self.ensure_head_init();
            let current_first = self.head_next();
            link.set_next(current_first);
            let link_nn = NonNull::new_unchecked(link as *const Link<T> as *mut Link<T>);
            self.set_head_next(Some(link_nn));
        }
    }

    /// Removes and returns a shared reference to the first element, or `None`
    /// if the list is empty.
    ///
    /// The removed node's link is marked [`LinkState::Free`] so it can be
    /// inserted into another list.
    pub fn pop_front(&mut self) -> Option<&T> {
        // SAFETY: We hold &mut self — exclusive access.
        // first_link_ptr points into a valid, live T by the caller's contract.
        // We advance the sentinel past the first node, then mark the removed
        // node Free before returning a reference to it (the T is still live;
        // only the link state changes).
        unsafe {
            let first_link_ptr = self.head_next()?;
            let second = first_link_ptr.as_ref().get_next();
            self.set_head_next(second);
            first_link_ptr.as_ref().set_free();
            Some(Self::link_ptr_to_ref(first_link_ptr))
        }
    }

    /// Removes a specific node from the list.
    ///
    /// If `node` is not in this list the call is a **no-op** (no panic).
    /// The removed node's link is marked [`LinkState::Free`].
    pub fn remove(&mut self, node: &T) {
        let target_link: *const Link<T> = node.link();

        // SAFETY: We hold &mut self — exclusive access.  We walk the list from
        // the sentinel using `prev` as a *const Link<T> (which we know is
        // actually &Link<T> with a lifetime at least as long as &mut self),
        // reading through UnsafeCell.  The cast from *const to *const is safe;
        // we only write to the sentinel/prev node's next field when we find the
        // target, and we write to the target's own next field via set_free.
        unsafe {
            self.ensure_head_init();

            // Start `prev` at the sentinel so removal of the first real node
            // needs no special case — the sentinel's next pointer is updated
            // exactly like any other predecessor's next pointer.
            let mut prev: *const Link<T> = &self.head as *const Link<T>;

            loop {
                let next_ptr = (*prev).get_next();
                match next_ptr {
                    None => return, // Reached end of list; node not found.
                    Some(curr_nn) => {
                        let curr: *const Link<T> = curr_nn.as_ptr();
                        if curr == target_link {
                            // Found the target — splice it out.
                            let after = (*curr).get_next();
                            (*prev).set_next(after);
                            (*curr).set_free();
                            return;
                        }
                        prev = curr;
                    }
                }
            }
        }
    }

    /// Returns `true` if `node` is currently in this list.
    ///
    /// Short-circuits immediately if the node's link is [`LinkState::Free`]
    /// (a free node cannot be in any list).
    pub fn contains(&self, node: &T) -> bool {
        let link = node.link();
        if !link.is_in_use() {
            return false;
        }
        let target: *const Link<T> = link;
        // SAFETY: Read-only walk; nodes are live by the caller's ownership
        // contract.  get_next reads through UnsafeCell with no concurrent mutation.
        unsafe {
            let mut cursor = self.head_next();
            while let Some(nn) = cursor {
                if core::ptr::eq(nn.as_ptr().cast::<Link<T>>(), target) {
                    return true;
                }
                cursor = nn.as_ref().get_next();
            }
        }
        false
    }

    /// Marks every node currently in the list as [`LinkState::Free`] and
    /// resets the list to empty.
    ///
    /// Mirrors `estd::forward_list::clear()` which walks the list calling
    /// `set_free()` on every node and then sets the sentinel's next to null.
    pub fn clear(&mut self) {
        // SAFETY: Exclusive access via &mut self.  We walk the chain and call
        // set_free on every link before resetting the sentinel.
        unsafe {
            let mut cursor = self.head_next();
            while let Some(nn) = cursor {
                let next = nn.as_ref().get_next();
                nn.as_ref().set_free();
                cursor = next;
            }
            self.set_head_next(None);
        }
    }

    /// Reverses the order of elements in place.
    ///
    /// Uses a standard iterative in-place reversal: for each node, redirect
    /// its `next` pointer at the accumulated reversed tail, then advance.
    /// Finally point the sentinel at the last node visited (which is now the
    /// new head).
    ///
    /// This is O(n) and requires no extra allocation.
    pub fn reverse(&mut self) {
        // SAFETY: Exclusive access via &mut self.  All pointer dereferences are
        // over nodes within the current list.  We rewrite each node's InList
        // next pointer, never touching Free nodes.  The sentinel is updated
        // at the end to point at the new first node.
        unsafe {
            let mut prev: Option<NonNull<Link<T>>> = None;
            let mut curr = self.head_next();

            while let Some(curr_nn) = curr {
                // Save the original next before overwriting it.
                let next = curr_nn.as_ref().get_next();
                // Redirect this node's next at the reversed tail so far.
                curr_nn.as_ref().set_next(prev);
                prev = curr;
                curr = next;
            }

            // `prev` is now the first node of the reversed list (or None if
            // the list was empty).
            self.set_head_next(prev);
        }
    }

    /// Returns an iterator over shared references to the elements of the list.
    pub fn iter(&self) -> Iter<'_, T> {
        // SAFETY: head_next is read-only.
        let first = unsafe { self.head_next() };
        Iter {
            current: first,
            _marker: PhantomData,
        }
    }
}

impl<T: Linked> Drop for ForwardList<T> {
    /// Marks all nodes still in the list as [`LinkState::Free`].
    ///
    /// Mirrors `~forward_list()` which calls `clear()`.  Without this, nodes
    /// would be left in `InList` state after the owning list is dropped,
    /// preventing them from being inserted into a new list.
    fn drop(&mut self) {
        self.clear();
    }
}

impl<T: Linked> Default for ForwardList<T> {
    fn default() -> Self {
        Self::new()
    }
}

// ForwardList is !Send and !Sync:
//   - !Send: PhantomData<*mut T> (raw pointer is !Send)
//   - !Sync: PhantomData<*mut T> (raw pointer is !Sync) + UnsafeCell in Link<T>
// Both auto-trait opt-outs are expressed by the PhantomData<*mut T> field.

// ──────────────────────────────────────────────────────────────────────────────
// Iter<'a, T>
// ──────────────────────────────────────────────────────────────────────────────

/// Iterator over shared references to elements of a [`ForwardList`].
///
/// Created by [`ForwardList::iter`].
pub struct Iter<'a, T: Linked> {
    current: Option<NonNull<Link<T>>>,
    _marker: PhantomData<&'a T>,
}

impl<'a, T: Linked> Iterator for Iter<'a, T> {
    type Item = &'a T;

    fn next(&mut self) -> Option<&'a T> {
        let nn = self.current?;
        // SAFETY: `nn` points to a Link<T> field inside a valid, live T.
        // The lifetime 'a is tied to the ForwardList borrow so no node can be
        // removed from the list while this iterator is alive.  We advance
        // `current` to the next link and return a reference to the current T.
        unsafe {
            self.current = nn.as_ref().get_next();
            Some(T::from_link_ptr(nn.as_ptr()).as_ref().unwrap_unchecked())
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Tests
// ──────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Test node type ────────────────────────────────────────────────────────

    struct TestNode {
        value: i32,
        link: Link<TestNode>,
    }

    impl TestNode {
        fn new(value: i32) -> Self {
            TestNode {
                value,
                link: Link::new(),
            }
        }
    }

    impl_linked!(TestNode, link);

    /// Collect up to N values from the list iterator into a fixed-size array.
    /// Returns (array, actual_count) — avoids heap allocation in no_std tests.
    fn collect_values<const N: usize>(list: &ForwardList<TestNode>) -> ([i32; N], usize) {
        let mut arr = [0i32; N];
        let mut count = 0usize;
        for node in list.iter() {
            if count < N {
                arr[count] = node.value;
            }
            count += 1;
        }
        (arr, count)
    }

    // ── Test 1: empty list ────────────────────────────────────────────────────

    #[test]
    fn test_empty_list() {
        let list: ForwardList<TestNode> = ForwardList::new();
        assert!(list.is_empty());
        assert_eq!(list.len(), 0);
        assert!(list.front().is_none());
        let mut iter = list.iter();
        assert!(iter.next().is_none());
    }

    // ── Test 2: push_front single element ────────────────────────────────────

    #[test]
    fn test_push_front_single() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n = TestNode::new(42);
        list.push_front(&n);

        assert!(!list.is_empty());
        assert_eq!(list.len(), 1);
        let front = list.front().expect("list should have a front element");
        assert_eq!(front.value, 42);
    }

    // ── Test 3: push_front multiple elements — verify LIFO order via iter ────

    #[test]
    fn test_push_front_lifo_order() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n1 = TestNode::new(1);
        let n2 = TestNode::new(2);
        let n3 = TestNode::new(3);

        list.push_front(&n1);
        list.push_front(&n2);
        list.push_front(&n3);

        assert_eq!(list.len(), 3);

        // push_front is LIFO so iteration order should be 3, 2, 1.
        let (vals, count) = collect_values::<3>(&list);
        assert_eq!(count, 3);
        assert_eq!(vals[0], 3);
        assert_eq!(vals[1], 2);
        assert_eq!(vals[2], 1);
    }

    // ── Test 4: pop_front returns elements in LIFO order ─────────────────────

    #[test]
    fn test_pop_front_lifo_order() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n1 = TestNode::new(10);
        let n2 = TestNode::new(20);
        let n3 = TestNode::new(30);

        list.push_front(&n1);
        list.push_front(&n2);
        list.push_front(&n3); // head: 30 → 20 → 10 → end

        let first = list.pop_front().expect("should pop 30");
        assert_eq!(first.value, 30);

        let second = list.pop_front().expect("should pop 20");
        assert_eq!(second.value, 20);

        let third = list.pop_front().expect("should pop 10");
        assert_eq!(third.value, 10);

        assert!(list.pop_front().is_none(), "list should now be empty");
        assert!(list.is_empty());
    }

    // ── Test 5: push_front is a no-op if node already in a list ──────────────

    #[test]
    fn test_push_front_noop_if_already_in_same_list() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n = TestNode::new(99);

        list.push_front(&n);
        assert_eq!(list.len(), 1);

        // Second push_front of the same node must be a no-op.
        list.push_front(&n);
        assert_eq!(list.len(), 1, "duplicate push_front must be ignored");

        let (vals, count) = collect_values::<4>(&list);
        assert_eq!(count, 1);
        assert_eq!(vals[0], 99);
    }

    #[test]
    fn test_push_front_noop_if_in_different_list() {
        let mut list1: ForwardList<TestNode> = ForwardList::new();
        let mut list2: ForwardList<TestNode> = ForwardList::new();
        let n = TestNode::new(7);

        list1.push_front(&n);
        // Attempt to push the same node into list2 — must be no-op (it is
        // already in_use in list1).
        list2.push_front(&n);

        assert_eq!(list1.len(), 1);
        assert_eq!(list2.len(), 0, "node already in list1 must not appear in list2");
    }

    // ── Test 6: remove from beginning, middle, end of 3-element list ─────────

    #[test]
    fn test_remove_from_beginning() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n1 = TestNode::new(1);
        let n2 = TestNode::new(2);
        let n3 = TestNode::new(3);
        list.push_front(&n1);
        list.push_front(&n2);
        list.push_front(&n3); // order: 3 → 2 → 1

        list.remove(&n3); // remove head

        let (vals, count) = collect_values::<3>(&list);
        assert_eq!(count, 2);
        assert_eq!(vals[0], 2);
        assert_eq!(vals[1], 1);
        assert!(!n3.link.is_in_use(), "removed node must be Free");
    }

    #[test]
    fn test_remove_from_middle() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n1 = TestNode::new(1);
        let n2 = TestNode::new(2);
        let n3 = TestNode::new(3);
        list.push_front(&n1);
        list.push_front(&n2);
        list.push_front(&n3); // order: 3 → 2 → 1

        list.remove(&n2); // remove middle

        let (vals, count) = collect_values::<3>(&list);
        assert_eq!(count, 2);
        assert_eq!(vals[0], 3);
        assert_eq!(vals[1], 1);
        assert!(!n2.link.is_in_use(), "removed node must be Free");
    }

    #[test]
    fn test_remove_from_end() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n1 = TestNode::new(1);
        let n2 = TestNode::new(2);
        let n3 = TestNode::new(3);
        list.push_front(&n1);
        list.push_front(&n2);
        list.push_front(&n3); // order: 3 → 2 → 1

        list.remove(&n1); // remove tail

        let (vals, count) = collect_values::<3>(&list);
        assert_eq!(count, 2);
        assert_eq!(vals[0], 3);
        assert_eq!(vals[1], 2);
        assert!(!n1.link.is_in_use(), "removed node must be Free");
    }

    // ── Test 7: remove node not in list — no-op ───────────────────────────────

    #[test]
    fn test_remove_not_in_list_noop() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n1 = TestNode::new(1);
        let n2 = TestNode::new(2); // never inserted

        list.push_front(&n1);
        list.remove(&n2); // must be a no-op

        assert_eq!(list.len(), 1);
        let (vals, count) = collect_values::<2>(&list);
        assert_eq!(count, 1);
        assert_eq!(vals[0], 1);
    }

    // ── Test 8: contains ──────────────────────────────────────────────────────

    #[test]
    fn test_contains_returns_true_for_members() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n1 = TestNode::new(1);
        let n2 = TestNode::new(2);
        let n3 = TestNode::new(3); // not inserted

        list.push_front(&n1);
        list.push_front(&n2);

        assert!(list.contains(&n1), "n1 is in the list");
        assert!(list.contains(&n2), "n2 is in the list");
        assert!(!list.contains(&n3), "n3 was never inserted");
    }

    // ── Test 9: clear marks all nodes as Free, list becomes empty ────────────

    #[test]
    fn test_clear() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n1 = TestNode::new(1);
        let n2 = TestNode::new(2);
        let n3 = TestNode::new(3);

        list.push_front(&n1);
        list.push_front(&n2);
        list.push_front(&n3);
        assert_eq!(list.len(), 3);

        list.clear();

        assert!(list.is_empty());
        assert_eq!(list.len(), 0);
        assert!(list.front().is_none());

        // Every node must be Free after clear.
        assert!(!n1.link.is_in_use(), "n1 must be Free after clear");
        assert!(!n2.link.is_in_use(), "n2 must be Free after clear");
        assert!(!n3.link.is_in_use(), "n3 must be Free after clear");

        // Nodes can be re-inserted after clear.
        list.push_front(&n1);
        assert_eq!(list.len(), 1);
    }

    // ── Test 10: reverse reverses the order ───────────────────────────────────

    #[test]
    fn test_reverse() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n1 = TestNode::new(1);
        let n2 = TestNode::new(2);
        let n3 = TestNode::new(3);

        list.push_front(&n1);
        list.push_front(&n2);
        list.push_front(&n3); // order: 3 → 2 → 1

        list.reverse(); // expected: 1 → 2 → 3

        let (vals, count) = collect_values::<3>(&list);
        assert_eq!(count, 3);
        assert_eq!(vals[0], 1);
        assert_eq!(vals[1], 2);
        assert_eq!(vals[2], 3);
    }

    #[test]
    fn test_reverse_empty_list() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        list.reverse(); // must not panic
        assert!(list.is_empty());
    }

    #[test]
    fn test_reverse_single_element() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n = TestNode::new(5);
        list.push_front(&n);
        list.reverse();
        assert_eq!(list.len(), 1);
        assert_eq!(list.front().unwrap().value, 5);
    }

    #[test]
    fn test_reverse_preserves_all_nodes_in_use() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let nodes: [TestNode; 5] = [
            TestNode::new(1),
            TestNode::new(2),
            TestNode::new(3),
            TestNode::new(4),
            TestNode::new(5),
        ];
        for n in &nodes {
            list.push_front(n); // list: 5 4 3 2 1
        }

        list.reverse(); // list: 1 2 3 4 5

        let (vals, count) = collect_values::<5>(&list);
        assert_eq!(count, 5);
        assert_eq!(vals, [1, 2, 3, 4, 5]);

        // All nodes remain in use after reverse.
        for n in &nodes {
            assert!(n.link.is_in_use(), "every node must still be in_use after reverse");
        }
    }

    // ── Test 11: is_in_use lifecycle ──────────────────────────────────────────

    #[test]
    fn test_is_in_use_before_push() {
        let n = TestNode::new(0);
        assert!(!n.link.is_in_use(), "fresh node must not be in use");
    }

    #[test]
    fn test_is_in_use_after_push_front() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n = TestNode::new(0);
        list.push_front(&n);
        assert!(n.link.is_in_use(), "node must be in_use after push_front");
    }

    #[test]
    fn test_is_in_use_after_pop_front() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n = TestNode::new(0);
        list.push_front(&n);
        let _ = list.pop_front();
        assert!(!n.link.is_in_use(), "node must be Free after pop_front");
    }

    #[test]
    fn test_is_in_use_after_clear() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n = TestNode::new(0);
        list.push_front(&n);
        list.clear();
        assert!(!n.link.is_in_use(), "node must be Free after clear");
    }

    #[test]
    fn test_is_in_use_after_remove() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n = TestNode::new(0);
        list.push_front(&n);
        list.remove(&n);
        assert!(!n.link.is_in_use(), "node must be Free after remove");
    }

    // ── Test 12: Drop of list marks all remaining nodes as Free ───────────────

    #[test]
    fn test_drop_marks_nodes_free() {
        let n1 = TestNode::new(1);
        let n2 = TestNode::new(2);

        {
            let mut list: ForwardList<TestNode> = ForwardList::new();
            list.push_front(&n1);
            list.push_front(&n2);
            assert!(n1.link.is_in_use());
            assert!(n2.link.is_in_use());
            // `list` is dropped here at end of inner scope.
        }

        assert!(!n1.link.is_in_use(), "n1 must be Free after list is dropped");
        assert!(!n2.link.is_in_use(), "n2 must be Free after list is dropped");
    }

    #[test]
    fn test_node_reusable_after_list_drop() {
        let n = TestNode::new(7);
        {
            let mut list: ForwardList<TestNode> = ForwardList::new();
            list.push_front(&n);
        } // list dropped — n.link transitions to Free

        let mut list2: ForwardList<TestNode> = ForwardList::new();
        list2.push_front(&n); // must succeed — node is Free again
        assert_eq!(list2.len(), 1);
        assert_eq!(list2.front().unwrap().value, 7);
    }

    // ── Test 13: multiple separate lists can coexist ──────────────────────────

    #[test]
    fn test_multiple_independent_lists_coexist() {
        let mut list_a: ForwardList<TestNode> = ForwardList::new();
        let mut list_b: ForwardList<TestNode> = ForwardList::new();

        let na = TestNode::new(100);
        let nb = TestNode::new(200);

        list_a.push_front(&na);
        list_b.push_front(&nb);

        assert_eq!(list_a.len(), 1);
        assert_eq!(list_b.len(), 1);

        assert!(list_a.contains(&na), "na should be in list_a");
        assert!(!list_a.contains(&nb), "nb should not be in list_a");
        assert!(list_b.contains(&nb), "nb should be in list_b");
        assert!(!list_b.contains(&na), "na should not be in list_b");

        assert_eq!(list_a.front().unwrap().value, 100);
        assert_eq!(list_b.front().unwrap().value, 200);
    }

    // ── Extra: pop_front on empty list returns None (not panic) ───────────────

    #[test]
    fn test_pop_front_empty_returns_none() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        assert!(list.pop_front().is_none());
        assert!(list.is_empty());
    }

    // ── Extra: iterating twice produces identical results ─────────────────────

    #[test]
    fn test_iter_is_idempotent() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n1 = TestNode::new(1);
        let n2 = TestNode::new(2);
        list.push_front(&n1);
        list.push_front(&n2); // order: 2 → 1

        let (vals1, c1) = collect_values::<2>(&list);
        let (vals2, c2) = collect_values::<2>(&list);
        assert_eq!(c1, c2);
        assert_eq!(vals1, vals2);
    }

    // ── Extra: re-insert node after removal works correctly ───────────────────

    #[test]
    fn test_reinsertion_after_removal() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n = TestNode::new(55);

        for _ in 0..3 {
            list.push_front(&n);
            assert_eq!(list.len(), 1, "list should have 1 element");
            list.remove(&n);
            assert_eq!(list.len(), 0, "list should be empty after remove");
            assert!(!n.link.is_in_use(), "node should be Free after remove");
        }
    }

    // ── Extra: remove only element leaves list empty ───────────────────────────

    #[test]
    fn test_remove_only_element() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n = TestNode::new(42);
        list.push_front(&n);
        list.remove(&n);
        assert!(list.is_empty());
        assert!(!n.link.is_in_use());
    }

    // ── Extra: len on list with many elements ──────────────────────────────────

    #[test]
    fn test_len_many_elements() {
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let nodes: [TestNode; 8] = [
            TestNode::new(0),
            TestNode::new(1),
            TestNode::new(2),
            TestNode::new(3),
            TestNode::new(4),
            TestNode::new(5),
            TestNode::new(6),
            TestNode::new(7),
        ];
        for n in &nodes {
            list.push_front(n);
        }
        assert_eq!(list.len(), 8);
        list.clear();
        assert_eq!(list.len(), 0);
    }

    // ── Extra: default() creates an empty list ────────────────────────────────

    #[test]
    fn test_default_creates_empty_list() {
        let list: ForwardList<TestNode> = ForwardList::default();
        assert!(list.is_empty());
        assert_eq!(list.len(), 0);
    }

    // ── Extra: contains on empty list returns false ────────────────────────────

    #[test]
    fn test_contains_empty_list() {
        let list: ForwardList<TestNode> = ForwardList::new();
        let n = TestNode::new(0);
        assert!(!list.contains(&n));
    }

    // ── Extra: offset_of correctness — value readable through iterator ────────

    #[test]
    fn test_offset_of_correctness_via_iter() {
        // If offset_of or from_link_ptr is wrong, node.value reads will be
        // garbage/UB.  Using distinctive sentinel values makes misalignment obvious.
        let mut list: ForwardList<TestNode> = ForwardList::new();
        let n1 = TestNode::new(0x0ABC_DEF0u32 as i32);
        let n2 = TestNode::new(0x1234_5678u32 as i32);
        list.push_front(&n1);
        list.push_front(&n2);

        let mut iter = list.iter();
        let first = iter.next().unwrap();
        let second = iter.next().unwrap();
        assert_eq!(first.value, 0x1234_5678u32 as i32);
        assert_eq!(second.value, 0x0ABC_DEF0u32 as i32);
    }
}
