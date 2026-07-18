//! Typed GPIO managers and deterministic debouncing (G07).

use bsw_time::{Duration, Instant};

pub trait DigitalInput {
    fn is_high(&self) -> bool;
}

pub trait DigitalOutput {
    fn set(&mut self, high: bool);
    fn is_set_high(&self) -> bool;
}

pub struct DebouncedInput<I: DigitalInput> {
    input: I,
    stable: bool,
    candidate: bool,
    candidate_since: Instant,
    debounce: Duration,
}

impl<I: DigitalInput> DebouncedInput<I> {
    pub fn new(input: I, debounce: Duration, now: Instant) -> Self {
        let initial = input.is_high();
        Self {
            input,
            stable: initial,
            candidate: initial,
            candidate_since: now,
            debounce,
        }
    }

    pub fn poll(&mut self, now: Instant) -> bool {
        let sample = self.input.is_high();
        if sample != self.candidate {
            self.candidate = sample;
            self.candidate_since = now;
        } else if sample != self.stable
            && now.is_at_or_after(self.candidate_since.wrapping_add(self.debounce))
        {
            self.stable = sample;
        }
        self.stable
    }

    pub fn input_mut(&mut self) -> &mut I {
        &mut self.input
    }
}

pub struct OutputManager<O: DigitalOutput> {
    output: O,
}

impl<O: DigitalOutput> OutputManager<O> {
    pub const fn new(output: O) -> Self {
        Self { output }
    }
    pub fn command(&mut self, high: bool) {
        self.output.set(high);
    }
    pub fn state(&self) -> bool {
        self.output.is_set_high()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::Cell;

    struct Input(Cell<bool>);
    impl DigitalInput for Input {
        fn is_high(&self) -> bool {
            self.0.get()
        }
    }

    #[test]
    fn debounce_changes_only_at_exact_injected_deadline() {
        let mut input = DebouncedInput::new(
            Input(Cell::new(false)),
            Duration::from_nanos(10),
            Instant::from_nanos(0),
        );
        input.input_mut().0.set(true);
        assert!(!input.poll(Instant::from_nanos(5)));
        assert!(!input.poll(Instant::from_nanos(14)));
        assert!(input.poll(Instant::from_nanos(15)));
    }
}
