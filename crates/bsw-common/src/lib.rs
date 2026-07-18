//! Small shared types used across the Rust BSW dependency graph.

#![cfg_attr(not(feature = "std"), no_std)]

use core::marker::PhantomData;

/// Common result code for interfaces that cannot carry a module-specific error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ResultCode {
    /// Operation completed successfully.
    Ok = 0,
    /// Operation is still pending.
    Pending = 1,
    /// A bounded resource has no remaining capacity.
    Full = 2,
    /// Input did not satisfy the interface contract.
    Invalid = 3,
    /// The requested object or operation is not available.
    Unavailable = 4,
    /// The operation failed for an implementation-specific reason.
    Failed = 5,
}

/// Error returned when a byte view exceeds its backing slice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ViewError;

/// Immutable byte view whose bounds are validated at construction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ByteView<'a> {
    bytes: &'a [u8],
}

impl<'a> ByteView<'a> {
    /// View an entire byte slice.
    pub const fn from_slice(bytes: &'a [u8]) -> Self {
        Self { bytes }
    }

    /// View `length` bytes beginning at `offset`.
    pub fn new(bytes: &'a [u8], offset: usize, length: usize) -> Result<Self, ViewError> {
        let end = offset.checked_add(length).ok_or(ViewError)?;
        Ok(Self {
            bytes: bytes.get(offset..end).ok_or(ViewError)?,
        })
    }

    /// Access the validated slice.
    pub const fn as_slice(self) -> &'a [u8] {
        self.bytes
    }

    /// Length of the view in bytes.
    pub const fn len(self) -> usize {
        self.bytes.len()
    }

    /// Whether the view contains no bytes.
    pub const fn is_empty(self) -> bool {
        self.bytes.is_empty()
    }
}

/// Mutable byte view whose bounds are validated at construction.
#[derive(Debug, PartialEq, Eq)]
pub struct ByteViewMut<'a> {
    bytes: &'a mut [u8],
}

impl<'a> ByteViewMut<'a> {
    /// View an entire mutable byte slice.
    pub fn from_slice(bytes: &'a mut [u8]) -> Self {
        Self { bytes }
    }

    /// View `length` mutable bytes beginning at `offset`.
    pub fn new(bytes: &'a mut [u8], offset: usize, length: usize) -> Result<Self, ViewError> {
        let end = offset.checked_add(length).ok_or(ViewError)?;
        Ok(Self {
            bytes: bytes.get_mut(offset..end).ok_or(ViewError)?,
        })
    }

    /// Access the validated slice.
    pub fn as_slice(&self) -> &[u8] {
        self.bytes
    }

    /// Mutably access the validated slice.
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        self.bytes
    }

    /// Length of the view in bytes.
    pub fn len(&self) -> usize {
        self.bytes.len()
    }

    /// Whether the view contains no bytes.
    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }
}

/// Strongly typed numeric identifier.
#[derive(Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Identifier<Tag> {
    raw: u16,
    marker: PhantomData<fn() -> Tag>,
}

impl<Tag> Identifier<Tag> {
    /// Create an identifier from its wire/configuration value.
    pub const fn new(raw: u16) -> Self {
        Self {
            raw,
            marker: PhantomData,
        }
    }

    /// Return the underlying numeric value.
    pub const fn get(self) -> u16 {
        self.raw
    }
}

impl<Tag> Clone for Identifier<Tag> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<Tag> Copy for Identifier<Tag> {}

/// Shared contract for state that can return to its initial condition.
pub trait Resettable {
    /// Restore the initial state.
    fn reset(&mut self);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn immutable_view_checks_overflow_and_bounds() {
        let bytes = [1, 2, 3, 4];
        assert_eq!(ByteView::new(&bytes, 1, 2).unwrap().as_slice(), &[2, 3]);
        assert_eq!(ByteView::new(&bytes, 3, 2), Err(ViewError));
        assert_eq!(ByteView::new(&bytes, usize::MAX, 2), Err(ViewError));
    }

    #[test]
    fn mutable_view_only_changes_selected_range() {
        let mut bytes = [1, 2, 3, 4];
        ByteViewMut::new(&mut bytes, 1, 2)
            .unwrap()
            .as_mut_slice()
            .fill(9);
        assert_eq!(bytes, [1, 9, 9, 4]);
    }

    #[test]
    fn identifier_tags_do_not_affect_representation() {
        enum Component {}
        let id = Identifier::<Component>::new(42);
        assert_eq!(id.get(), 42);
        assert_eq!(core::mem::size_of_val(&id), core::mem::size_of::<u16>());
    }
}
