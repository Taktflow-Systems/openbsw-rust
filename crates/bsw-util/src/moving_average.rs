//! Fixed-window moving average filter.
//!
//! Rust port of the C++ `util/math/MovingAverage` template.
//!
//! A circular buffer keeps the last `N` samples and maintains a running sum
//! so that [`MovingAverage::average`] is always O(1).  The buffer is
//! pre-filled with the `initial` value supplied at construction, matching the
//! C++ behaviour: `average()` returns `initial` immediately after `new()`.
//!
//! # Quick start
//!
//! ```rust
//! use bsw_util::moving_average::MovingAverage;
//!
//! // 4-sample integer moving average, pre-filled with 0
//! let mut ma: MovingAverage<i32, 4> = MovingAverage::zero();
//! ma.add(10);
//! ma.add(20);
//! // buffer is [0, 0, 10, 20], sum = 30, average = 30/4 = 7
//! assert_eq!(ma.average(), 7);
//!
//! // 3-sample f32 moving average, pre-filled with 1.0
//! let mut maf: MovingAverage<f32, 3> = MovingAverage::new(1.0);
//! maf.add(4.0);
//! // buffer is [1.0, 1.0, 4.0], sum = 6.0, average = 2.0
//! assert!((maf.average() - 2.0_f32).abs() < f32::EPSILON);
//! ```

// ---------------------------------------------------------------------------
// Averageable trait
// ---------------------------------------------------------------------------

/// Types that can be used as moving-average samples.
///
/// Requires copy semantics, the four arithmetic operations needed for the
/// running-sum algorithm, and a way to convert a `usize` denominator into `Self`.
pub trait Averageable:
    Copy
    + core::ops::Add<Output = Self>
    + core::ops::Sub<Output = Self>
    + core::ops::Div<Output = Self>
    + Default
{
    /// Convert a `usize` (the window size or current count) into `Self`.
    ///
    /// Used as the divisor in [`MovingAverage::average`].
    fn from_usize(n: usize) -> Self;
}

macro_rules! impl_averageable_int {
    ($($t:ty),+) => {
        $(
            impl Averageable for $t {
                #[inline]
                #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap, clippy::cast_sign_loss)]
                fn from_usize(n: usize) -> Self {
                    n as Self
                }
            }
        )+
    };
}

macro_rules! impl_averageable_float {
    ($($t:ty),+) => {
        $(
            impl Averageable for $t {
                #[inline]
                fn from_usize(n: usize) -> Self {
                    n as Self
                }
            }
        )+
    };
}

impl_averageable_int!(u8, u16, u32, u64, usize, i8, i16, i32, i64, isize);
impl_averageable_float!(f32, f64);

// ---------------------------------------------------------------------------
// MovingAverage<T, N>
// ---------------------------------------------------------------------------

/// Fixed-window moving average filter.
///
/// Stores the last `N` samples in a circular buffer and exposes O(1)
/// [`average`](MovingAverage::average) via a maintained running sum.
///
/// All slots are initialised to `initial` at construction, so
/// `average()` returns `initial` before any samples are added — matching
/// the C++ `MovingAverage` behaviour.
///
/// # Type parameters
///
/// - `T` — sample type; must implement [`Averageable`]
/// - `N` — window size (number of slots); must be `>= 1`
///
/// # Panics
///
/// [`MovingAverage::new`] and [`MovingAverage::zero`] panic at runtime if
/// `N == 0` (compile-time const-assert is not yet stable for generics).
pub struct MovingAverage<T, const N: usize> {
    /// Circular sample buffer, length N.
    buffer: [T; N],
    /// Next write position (wraps at N).
    index: usize,
    /// Running sum of all N buffer slots.
    sum: T,
    /// Number of real samples added (capped at N; used for `is_filled`).
    count: usize,
}

impl<T: Averageable, const N: usize> MovingAverage<T, N> {
    // -----------------------------------------------------------------------
    // Constructors
    // -----------------------------------------------------------------------

    /// Create a new moving average with all `N` slots initialised to `initial`.
    ///
    /// `average()` returns `initial` immediately because the running sum is
    /// `N * initial` and dividing by `N` recovers `initial`.
    ///
    /// # Panics
    ///
    /// Panics if `N == 0`.
    pub fn new(initial: T) -> Self {
        assert!(N >= 1, "MovingAverage window size N must be >= 1");

        // Compute sum = N * initial via repeated addition (avoids a Mul bound).
        let mut sum = T::default();
        for _ in 0..N {
            sum = sum + initial;
        }

        Self {
            buffer: [initial; N],
            index: 0,
            sum,
            count: 0,
        }
    }

    /// Create a new moving average with all slots initialised to `T::default()` (zero).
    ///
    /// Equivalent to `MovingAverage::new(T::default())`.
    ///
    /// # Panics
    ///
    /// Panics if `N == 0`.
    pub fn zero() -> Self {
        Self::new(T::default())
    }

    // -----------------------------------------------------------------------
    // Mutation
    // -----------------------------------------------------------------------

    /// Add a new sample, overwriting the oldest slot in the circular buffer.
    ///
    /// The running sum is updated in O(1) by subtracting the evicted value
    /// and adding the new one.
    pub fn add(&mut self, sample: T) {
        // Remove the outgoing slot from the sum.
        self.sum = self.sum - self.buffer[self.index];
        // Store and account for the new sample.
        self.buffer[self.index] = sample;
        self.sum = self.sum + sample;
        // Advance the write pointer.
        self.index = (self.index + 1) % N;
        // Track real additions, capped at N.
        if self.count < N {
            self.count += 1;
        }
    }

    /// Reset all buffer slots to `value` and clear the sample counter.
    ///
    /// After reset, `average()` returns `value` and `is_filled()` returns `false`.
    pub fn reset(&mut self, value: T) {
        for slot in &mut self.buffer {
            *slot = value;
        }
        self.index = 0;
        self.count = 0;
        // Recompute sum = N * value.
        let mut sum = T::default();
        for _ in 0..N {
            sum = sum + value;
        }
        self.sum = sum;
    }

    // -----------------------------------------------------------------------
    // Queries
    // -----------------------------------------------------------------------

    /// Returns the current moving average.
    ///
    /// Always divides the running sum by `N` (the full window size), matching
    /// C++ behaviour where the buffer is pre-filled at construction.
    ///
    /// For integer types this performs truncating integer division.
    pub fn average(&self) -> T {
        self.sum / T::from_usize(N)
    }

    /// Returns the current running sum of all `N` buffer slots.
    pub fn sum(&self) -> T {
        self.sum
    }

    /// Returns the number of real samples that have been added via [`add`](Self::add).
    ///
    /// Saturates at `N` — once the buffer has been fully filled this always
    /// returns `N`.
    pub fn count(&self) -> usize {
        self.count
    }

    /// Returns `true` once at least `N` samples have been added.
    pub fn is_filled(&self) -> bool {
        self.count >= N
    }

    /// The window size (same as the const generic `N`).
    pub const fn window_size() -> usize {
        N
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // 1. New with initial value — average returns initial
    // -----------------------------------------------------------------------

    #[test]
    fn new_returns_initial_value() {
        let ma: MovingAverage<i32, 4> = MovingAverage::new(7);
        assert_eq!(ma.average(), 7);
    }

    #[test]
    fn new_f32_returns_initial_value() {
        let ma: MovingAverage<f32, 4> = MovingAverage::new(3.5);
        assert!((ma.average() - 3.5_f32).abs() < f32::EPSILON);
    }

    // -----------------------------------------------------------------------
    // 2. Add one sample — average is (initial*(N-1) + sample) / N
    // -----------------------------------------------------------------------

    #[test]
    fn add_one_sample_integer() {
        // N=4, initial=0, add 8 → buffer=[8,0,0,0], sum=8, avg=8/4=2
        let mut ma: MovingAverage<i32, 4> = MovingAverage::zero();
        ma.add(8);
        assert_eq!(ma.average(), 2);
    }

    #[test]
    fn add_one_sample_with_nonzero_initial() {
        // N=4, initial=4, add 8 → buffer=[8,4,4,4], sum=20, avg=20/4=5
        let mut ma: MovingAverage<i32, 4> = MovingAverage::new(4);
        ma.add(8);
        assert_eq!(ma.average(), 5);
    }

    // -----------------------------------------------------------------------
    // 3. Fill completely with the same value — average equals that value
    // -----------------------------------------------------------------------

    #[test]
    fn fill_with_same_value() {
        let mut ma: MovingAverage<i32, 4> = MovingAverage::zero();
        for _ in 0..4 {
            ma.add(10);
        }
        assert_eq!(ma.average(), 10);
    }

    // -----------------------------------------------------------------------
    // 4. Fill with sequential values — verify average
    // -----------------------------------------------------------------------

    #[test]
    fn sequential_values_average() {
        // N=4, add 1,2,3,4 → sum=10, avg=10/4=2 (integer truncation)
        let mut ma: MovingAverage<i32, 4> = MovingAverage::zero();
        for i in 1..=4_i32 {
            ma.add(i);
        }
        assert_eq!(ma.average(), 2); // 10/4 = 2 (truncated)
    }

    #[test]
    fn sequential_values_f32_average() {
        // N=4, add 1.0,2.0,3.0,4.0 → sum=10.0, avg=2.5
        let mut ma: MovingAverage<f32, 4> = MovingAverage::zero();
        for i in 1..=4 {
            ma.add(i as f32);
        }
        assert!((ma.average() - 2.5_f32).abs() < f32::EPSILON);
    }

    // -----------------------------------------------------------------------
    // 5. Wrap-around — oldest sample is dropped
    // -----------------------------------------------------------------------

    #[test]
    fn wrap_around_drops_oldest() {
        // N=3, zero-init.
        // add 10,20,30 → buffer full: [10,20,30], sum=60, avg=20
        // add 40 → evicts 10:  [40,20,30], sum=90, avg=30
        let mut ma: MovingAverage<i32, 3> = MovingAverage::zero();
        ma.add(10);
        ma.add(20);
        ma.add(30);
        assert_eq!(ma.average(), 20);
        ma.add(40);
        assert_eq!(ma.average(), 30);
    }

    #[test]
    fn wrap_around_many_times() {
        // N=3.  Push 100 identical values = 50, check the final average = 50.
        let mut ma: MovingAverage<i32, 3> = MovingAverage::zero();
        for _ in 0..100 {
            ma.add(50);
        }
        assert_eq!(ma.average(), 50);
    }

    // -----------------------------------------------------------------------
    // 6. Reset restores initial state
    // -----------------------------------------------------------------------

    #[test]
    fn reset_restores_state() {
        let mut ma: MovingAverage<i32, 4> = MovingAverage::zero();
        ma.add(100);
        ma.add(200);
        ma.reset(5);
        assert_eq!(ma.average(), 5);
        assert_eq!(ma.count(), 0);
        assert!(!ma.is_filled());
    }

    #[test]
    fn reset_allows_normal_operation_after() {
        let mut ma: MovingAverage<i32, 3> = MovingAverage::zero();
        ma.add(1);
        ma.add(2);
        ma.add(3);
        ma.reset(0);
        ma.add(9);
        // buffer=[9,0,0], sum=9, avg=9/3=3
        assert_eq!(ma.average(), 3);
    }

    // -----------------------------------------------------------------------
    // 7. Window size returns N
    // -----------------------------------------------------------------------

    #[test]
    fn window_size_returns_n() {
        assert_eq!(MovingAverage::<i32, 8>::window_size(), 8);
        assert_eq!(MovingAverage::<f64, 16>::window_size(), 16);
        assert_eq!(MovingAverage::<u8, 1>::window_size(), 1);
    }

    // -----------------------------------------------------------------------
    // 8. is_filled after N additions
    // -----------------------------------------------------------------------

    #[test]
    fn is_filled_after_n_additions() {
        let mut ma: MovingAverage<i32, 3> = MovingAverage::zero();
        assert!(!ma.is_filled());
        ma.add(1);
        assert!(!ma.is_filled());
        ma.add(2);
        assert!(!ma.is_filled());
        ma.add(3);
        assert!(ma.is_filled());
        ma.add(4); // still filled
        assert!(ma.is_filled());
    }

    // -----------------------------------------------------------------------
    // 9. f32 average with fractional results
    // -----------------------------------------------------------------------

    #[test]
    fn f32_fractional_average() {
        // N=2, add 1.0 and 2.0 → avg = 1.5
        let mut ma: MovingAverage<f32, 2> = MovingAverage::zero();
        ma.add(1.0);
        ma.add(2.0);
        assert!((ma.average() - 1.5_f32).abs() < f32::EPSILON);
    }

    #[test]
    fn f32_mixed_values_average() {
        // N=4, add 1.1, 2.2, 3.3, 4.4 → sum=11.0, avg=2.75
        let mut ma: MovingAverage<f32, 4> = MovingAverage::zero();
        ma.add(1.1);
        ma.add(2.2);
        ma.add(3.3);
        ma.add(4.4);
        assert!((ma.average() - 2.75_f32).abs() < 1e-5_f32);
    }

    // -----------------------------------------------------------------------
    // 10. N=1 — single-slot window (last sample == average)
    // -----------------------------------------------------------------------

    #[test]
    fn n1_window_last_sample_is_average() {
        let mut ma: MovingAverage<i32, 1> = MovingAverage::zero();
        ma.add(42);
        assert_eq!(ma.average(), 42);
        ma.add(99);
        assert_eq!(ma.average(), 99);
    }

    #[test]
    fn n1_window_initial_value() {
        let ma: MovingAverage<i32, 1> = MovingAverage::new(7);
        assert_eq!(ma.average(), 7);
    }

    // -----------------------------------------------------------------------
    // 11. N=2 minimum useful window
    // -----------------------------------------------------------------------

    #[test]
    fn n2_window_basic() {
        let mut ma: MovingAverage<i32, 2> = MovingAverage::zero();
        ma.add(10);
        // buffer=[10,0], sum=10, avg=5
        assert_eq!(ma.average(), 5);
        ma.add(20);
        // buffer=[10,20], sum=30, avg=15
        assert_eq!(ma.average(), 15);
        ma.add(30);
        // evicts 10: buffer=[20,30], sum=50, avg=25
        assert_eq!(ma.average(), 25);
    }

    // -----------------------------------------------------------------------
    // 12. Large window N=256
    // -----------------------------------------------------------------------

    #[test]
    fn large_window_n256() {
        let mut ma: MovingAverage<i32, 256> = MovingAverage::zero();
        for _ in 0..256 {
            ma.add(100);
        }
        assert_eq!(ma.average(), 100);
        assert!(ma.is_filled());
        assert_eq!(ma.count(), 256);
    }

    #[test]
    fn large_window_n256_partial_fill() {
        // Fill 100 of 256 slots with 256, rest remain 0.
        // sum = 100*256 = 25600, avg = 25600/256 = 100
        let mut ma: MovingAverage<i32, 256> = MovingAverage::zero();
        for _ in 0..100 {
            ma.add(256);
        }
        assert_eq!(ma.average(), 100);
        assert!(!ma.is_filled());
    }

    // -----------------------------------------------------------------------
    // 13. Integer truncation behaviour
    // -----------------------------------------------------------------------

    #[test]
    fn integer_truncation_division() {
        // N=4, add 1,1,1,2 → sum=5, avg=5/4=1 (truncated)
        let mut ma: MovingAverage<i32, 4> = MovingAverage::zero();
        ma.add(1);
        ma.add(1);
        ma.add(1);
        ma.add(2);
        assert_eq!(ma.average(), 1);
    }

    #[test]
    fn integer_truncation_u8() {
        // N=3, add 10,10,11 → sum=31, avg=31/3=10 (truncated)
        let mut ma: MovingAverage<u8, 3> = MovingAverage::zero();
        ma.add(10);
        ma.add(10);
        ma.add(11);
        assert_eq!(ma.average(), 10_u8);
    }

    // -----------------------------------------------------------------------
    // 14. sum() returns correct running total
    // -----------------------------------------------------------------------

    #[test]
    fn sum_tracks_all_slots() {
        // N=4, zero-init → sum=0
        let mut ma: MovingAverage<i32, 4> = MovingAverage::zero();
        assert_eq!(ma.sum(), 0);
        ma.add(5);
        // buffer=[5,0,0,0], sum=5
        assert_eq!(ma.sum(), 5);
        ma.add(10);
        // buffer=[5,10,0,0], sum=15
        assert_eq!(ma.sum(), 15);
    }

    #[test]
    fn sum_after_wrap() {
        // N=2, add 10,20,30.
        // after [10,20]: sum=30
        // after add 30 (evicts 10): sum=50
        let mut ma: MovingAverage<i32, 2> = MovingAverage::zero();
        ma.add(10);
        ma.add(20);
        assert_eq!(ma.sum(), 30);
        ma.add(30);
        assert_eq!(ma.sum(), 50);
    }

    // -----------------------------------------------------------------------
    // 15. count() tracks additions correctly
    // -----------------------------------------------------------------------

    #[test]
    fn count_increments_and_caps() {
        let mut ma: MovingAverage<i32, 4> = MovingAverage::zero();
        assert_eq!(ma.count(), 0);
        ma.add(1);
        assert_eq!(ma.count(), 1);
        ma.add(2);
        assert_eq!(ma.count(), 2);
        ma.add(3);
        assert_eq!(ma.count(), 3);
        ma.add(4);
        assert_eq!(ma.count(), 4);
        ma.add(5); // beyond N — stays at N
        assert_eq!(ma.count(), 4);
        ma.add(6);
        assert_eq!(ma.count(), 4);
    }

    // -----------------------------------------------------------------------
    // 16. Zero-initialised via zero()
    // -----------------------------------------------------------------------

    #[test]
    fn zero_initialised_average_is_zero() {
        let ma: MovingAverage<i32, 5> = MovingAverage::zero();
        assert_eq!(ma.average(), 0);
        assert_eq!(ma.sum(), 0);
        assert_eq!(ma.count(), 0);
    }

    #[test]
    fn zero_f64_average_is_zero() {
        let ma: MovingAverage<f64, 8> = MovingAverage::zero();
        assert!((ma.average()).abs() < f64::EPSILON);
    }

    // -----------------------------------------------------------------------
    // 17. Alternating values
    // -----------------------------------------------------------------------

    #[test]
    fn alternating_values_n4() {
        // N=4, alternating 0 and 100 across many rounds.
        // After steady state: two slots 0, two slots 100 → avg = 50
        let mut ma: MovingAverage<i32, 4> = MovingAverage::zero();
        for _ in 0..20 {
            ma.add(0);
            ma.add(100);
        }
        assert_eq!(ma.average(), 50);
    }

    #[test]
    fn alternating_values_n3() {
        // N=3, alternating 0 and 60.
        // After many pairs the last 3 values are either [0,60,0] or [60,0,60].
        let mut ma: MovingAverage<i32, 3> = MovingAverage::zero();
        for _ in 0..9 {
            ma.add(0);
            ma.add(60);
        }
        let avg = ma.average();
        assert!(avg == 20 || avg == 40, "unexpected average {avg}");
    }
}
