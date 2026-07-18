//! Bounded console front end for the OpenBSW Rust port (package C15).
//!
//! The console mirrors the upstream `util::command` / console model with
//! native Rust ownership:
//!
//! - [`LineEditor`] turns a raw byte stream into complete lines with
//!   backspace editing, CR/LF/CRLF handling, and hard input bounds.
//! - [`tokenize`] splits a line into a fixed-capacity argument list.
//! - [`Command`] and [`Console`] provide a heap-free command registry with a
//!   built-in `help`, argument forwarding, and uniform error reporting.
//!
//! Every input path is bounded: an over-long line, an invalid byte
//! sequence, or too many arguments produces a diagnostic instead of
//! unbounded buffering or a panic.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod chario;
#[cfg(feature = "std")]
pub mod posix;
pub mod task;

use core::fmt::Write;

/// Maximum number of whitespace-separated tokens per line, including the
/// command name itself. Mirrors upstream's small fixed argv.
pub const MAX_TOKENS: usize = 8;

/// Completed input event produced by [`LineEditor::feed`].
#[derive(Debug, PartialEq, Eq)]
pub enum LineEvent<'a> {
    /// A complete line of valid UTF-8 input, without its terminator.
    Line(&'a str),
    /// The line exceeded the editor capacity and was discarded.
    Overflow,
    /// The line contained invalid UTF-8 and was discarded.
    Invalid,
}

/// Fixed-capacity line editor.
///
/// Accepts one byte at a time and reports a [`LineEvent`] whenever a line
/// terminator arrives. Backspace (`0x08`) and DEL (`0x7F`) erase the last
/// byte; other control bytes are ignored. CRLF is treated as one
/// terminator; a bare CR or LF also completes a line.
pub struct LineEditor<const N: usize> {
    buffer: [u8; N],
    len: usize,
    overflow: bool,
    last_was_cr: bool,
    consumed: bool,
}

impl<const N: usize> LineEditor<N> {
    /// Create an empty editor.
    pub const fn new() -> Self {
        Self {
            buffer: [0; N],
            len: 0,
            overflow: false,
            last_was_cr: false,
            consumed: false,
        }
    }

    /// Number of buffered bytes in the current, unfinished line.
    pub fn pending_len(&self) -> usize {
        if self.consumed {
            0
        } else {
            self.len
        }
    }

    /// Feed one input byte; returns an event when a line completes.
    pub fn feed(&mut self, byte: u8) -> Option<LineEvent<'_>> {
        if self.consumed {
            self.len = 0;
            self.overflow = false;
            self.consumed = false;
        }
        let was_cr = core::mem::take(&mut self.last_was_cr);
        match byte {
            b'\r' => {
                self.last_was_cr = true;
                Some(self.complete())
            }
            b'\n' => {
                if was_cr {
                    // Second half of CRLF: already reported at the CR.
                    None
                } else {
                    Some(self.complete())
                }
            }
            0x08 | 0x7F => {
                if self.len > 0 {
                    self.len -= 1;
                }
                None
            }
            0x00..=0x1F => None,
            _ => {
                if self.len == N {
                    self.overflow = true;
                } else {
                    self.buffer[self.len] = byte;
                    self.len += 1;
                }
                None
            }
        }
    }

    fn complete(&mut self) -> LineEvent<'_> {
        self.consumed = true;
        if self.overflow {
            return LineEvent::Overflow;
        }
        match core::str::from_utf8(&self.buffer[..self.len]) {
            Ok(line) => LineEvent::Line(line),
            Err(_) => LineEvent::Invalid,
        }
    }
}

impl<const N: usize> Default for LineEditor<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Fixed-capacity token list borrowed from one input line.
#[derive(Debug, PartialEq, Eq)]
pub struct Tokens<'a> {
    tokens: [&'a str; MAX_TOKENS],
    count: usize,
}

impl<'a> Tokens<'a> {
    /// All tokens in input order.
    pub fn as_slice(&self) -> &[&'a str] {
        &self.tokens[..self.count]
    }

    /// Number of tokens.
    pub fn len(&self) -> usize {
        self.count
    }

    /// Whether the line contained no tokens.
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }
}

/// Tokenization failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TokenizeError {
    /// The line contains more than [`MAX_TOKENS`] tokens.
    TooManyTokens,
}

/// Split a line on ASCII whitespace into at most [`MAX_TOKENS`] tokens.
pub fn tokenize(line: &str) -> Result<Tokens<'_>, TokenizeError> {
    let mut tokens = Tokens {
        tokens: [""; MAX_TOKENS],
        count: 0,
    };
    for token in line.split_ascii_whitespace() {
        if tokens.count == MAX_TOKENS {
            return Err(TokenizeError::TooManyTokens);
        }
        tokens.tokens[tokens.count] = token;
        tokens.count += 1;
    }
    Ok(tokens)
}

/// Result of one command execution.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandStatus {
    /// The command completed.
    Ok,
    /// The arguments were unusable; the console prints the usage line.
    UsageError,
    /// The command failed; it has already reported details to the output.
    Error,
}

/// One console command.
pub trait Command {
    /// Unique lowercase command name.
    fn name(&self) -> &'static str;

    /// One-line usage/help text.
    fn help(&self) -> &'static str;

    /// Execute with the arguments after the command name.
    fn execute(&mut self, args: &[&str], out: &mut dyn Write) -> CommandStatus;
}

/// Registration failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegisterError {
    /// The registry is full.
    Full,
    /// A command with the same name is already registered.
    DuplicateName,
}

/// Bounded command console: line editor plus command registry.
///
/// `CMDS` bounds the registry; `LINE` bounds one input line.
pub struct Console<'a, const CMDS: usize, const LINE: usize> {
    editor: LineEditor<LINE>,
    commands: [Option<&'a mut dyn Command>; CMDS],
    count: usize,
}

impl<'a, const CMDS: usize, const LINE: usize> Console<'a, CMDS, LINE> {
    /// Create a console with an empty registry.
    pub fn new() -> Self {
        Self {
            editor: LineEditor::new(),
            commands: [const { None }; CMDS],
            count: 0,
        }
    }

    /// Register a command.
    pub fn register(&mut self, command: &'a mut dyn Command) -> Result<(), RegisterError> {
        if self.count == CMDS {
            return Err(RegisterError::Full);
        }
        let name = command.name();
        if self.commands[..self.count].iter().any(|slot| {
            slot.as_ref()
                .is_some_and(|existing| existing.name() == name)
        }) {
            return Err(RegisterError::DuplicateName);
        }
        self.commands[self.count] = Some(command);
        self.count += 1;
        Ok(())
    }

    /// Number of registered commands.
    pub fn command_count(&self) -> usize {
        self.count
    }

    /// Feed one input byte, executing a command when a line completes.
    pub fn feed(&mut self, byte: u8, out: &mut dyn Write) {
        let commands = &mut self.commands[..self.count];
        match self.editor.feed(byte) {
            None => {}
            Some(LineEvent::Overflow) => {
                let _ = writeln!(out, "error: line too long");
            }
            Some(LineEvent::Invalid) => {
                let _ = writeln!(out, "error: invalid input");
            }
            Some(LineEvent::Line(line)) => Self::dispatch(commands, line, out),
        }
    }

    /// Feed a whole byte sequence.
    pub fn feed_slice(&mut self, bytes: &[u8], out: &mut dyn Write) {
        for &byte in bytes {
            self.feed(byte, out);
        }
    }

    fn dispatch(commands: &mut [Option<&'a mut dyn Command>], line: &str, out: &mut dyn Write) {
        let tokens = match tokenize(line) {
            Ok(tokens) => tokens,
            Err(TokenizeError::TooManyTokens) => {
                let _ = writeln!(out, "error: too many arguments");
                return;
            }
        };
        let Some((&name, args)) = tokens.as_slice().split_first() else {
            return;
        };
        if name == "help" {
            Self::help(commands, args, out);
            return;
        }
        for slot in commands.iter_mut() {
            let Some(command) = slot.as_mut() else {
                continue;
            };
            if command.name() == name {
                match command.execute(args, out) {
                    CommandStatus::Ok | CommandStatus::Error => {}
                    CommandStatus::UsageError => {
                        let _ = writeln!(out, "usage: {}", command.help());
                    }
                }
                return;
            }
        }
        let _ = writeln!(out, "unknown command: {name} (try 'help')");
    }

    fn help(commands: &mut [Option<&'a mut dyn Command>], args: &[&str], out: &mut dyn Write) {
        if let Some(&wanted) = args.first() {
            for slot in commands.iter() {
                if let Some(command) = slot.as_ref() {
                    if command.name() == wanted {
                        let _ = writeln!(out, "{}", command.help());
                        return;
                    }
                }
            }
            let _ = writeln!(out, "unknown command: {wanted}");
            return;
        }
        let _ = writeln!(out, "help - list commands");
        for slot in commands.iter() {
            if let Some(command) = slot.as_ref() {
                let _ = writeln!(out, "{} - {}", command.name(), command.help());
            }
        }
    }
}

impl<const CMDS: usize, const LINE: usize> Default for Console<'_, CMDS, LINE> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Echo {
        calls: u32,
    }

    impl Command for Echo {
        fn name(&self) -> &'static str {
            "echo"
        }

        fn help(&self) -> &'static str {
            "echo <words..> - print the arguments"
        }

        fn execute(&mut self, args: &[&str], out: &mut dyn Write) -> CommandStatus {
            self.calls += 1;
            if args.is_empty() {
                return CommandStatus::UsageError;
            }
            let mut first = true;
            for arg in args {
                if !first {
                    let _ = write!(out, " ");
                }
                let _ = write!(out, "{arg}");
                first = false;
            }
            let _ = writeln!(out);
            CommandStatus::Ok
        }
    }

    struct Fail;

    impl Command for Fail {
        fn name(&self) -> &'static str {
            "fail"
        }

        fn help(&self) -> &'static str {
            "fail - always fails"
        }

        fn execute(&mut self, _args: &[&str], out: &mut dyn Write) -> CommandStatus {
            let _ = writeln!(out, "fail: broken as designed");
            CommandStatus::Error
        }
    }

    fn run_script<const CMDS: usize>(console: &mut Console<'_, CMDS, 32>, script: &str) -> String {
        let mut out = String::new();
        console.feed_slice(script.as_bytes(), &mut out);
        out
    }

    #[test]
    fn line_editor_handles_backspace_and_terminators() {
        let mut editor: LineEditor<8> = LineEditor::new();
        for &byte in b"abX\x08c" {
            assert_eq!(editor.feed(byte), None);
        }
        assert_eq!(editor.feed(b'\n'), Some(LineEvent::Line("abc")));
        // CRLF reports once, at the CR.
        for &byte in b"xy" {
            editor.feed(byte);
        }
        assert_eq!(editor.feed(b'\r'), Some(LineEvent::Line("xy")));
        assert_eq!(editor.feed(b'\n'), None);
        // Bare CR still completes the next line.
        editor.feed(b'z');
        assert_eq!(editor.feed(b'\r'), Some(LineEvent::Line("z")));
    }

    #[test]
    fn line_editor_bounds_input_and_recovers() {
        let mut editor: LineEditor<4> = LineEditor::new();
        for &byte in b"toolong" {
            editor.feed(byte);
        }
        assert_eq!(editor.feed(b'\n'), Some(LineEvent::Overflow));
        editor.feed(b'o');
        editor.feed(b'k');
        assert_eq!(editor.feed(b'\n'), Some(LineEvent::Line("ok")));
    }

    #[test]
    fn line_editor_rejects_invalid_utf8() {
        let mut editor: LineEditor<8> = LineEditor::new();
        editor.feed(0xFF);
        assert_eq!(editor.feed(b'\n'), Some(LineEvent::Invalid));
        editor.feed(b'a');
        assert_eq!(editor.feed(b'\n'), Some(LineEvent::Line("a")));
    }

    #[test]
    fn backspace_at_start_and_control_bytes_are_ignored() {
        let mut editor: LineEditor<8> = LineEditor::new();
        editor.feed(0x08);
        editor.feed(0x1B);
        editor.feed(b'a');
        assert_eq!(editor.feed(b'\n'), Some(LineEvent::Line("a")));
    }

    #[test]
    fn tokenizer_bounds_and_splits() {
        let tokens = tokenize("  one   two three ").unwrap();
        assert_eq!(tokens.as_slice(), ["one", "two", "three"]);
        assert!(tokenize("").unwrap().is_empty());
        assert_eq!(
            tokenize("1 2 3 4 5 6 7 8 9"),
            Err(TokenizeError::TooManyTokens)
        );
        let max = tokenize("1 2 3 4 5 6 7 8").unwrap();
        assert_eq!(max.len(), MAX_TOKENS);
    }

    #[test]
    fn console_executes_commands_with_arguments() {
        let mut echo = Echo { calls: 0 };
        let mut console: Console<'_, 4, 32> = Console::new();
        console.register(&mut echo).unwrap();
        let out = run_script(&mut console, "echo hello world\n");
        assert_eq!(out, "hello world\n");
    }

    #[test]
    fn console_reports_usage_unknown_and_failures() {
        let mut echo = Echo { calls: 0 };
        let mut fail = Fail;
        let mut console: Console<'_, 4, 32> = Console::new();
        console.register(&mut echo).unwrap();
        console.register(&mut fail).unwrap();
        let out = run_script(&mut console, "echo\nnope\nfail\n");
        assert_eq!(
            out,
            "usage: echo <words..> - print the arguments\n\
             unknown command: nope (try 'help')\n\
             fail: broken as designed\n"
        );
    }

    #[test]
    fn console_help_lists_and_describes_commands() {
        let mut echo = Echo { calls: 0 };
        let mut console: Console<'_, 4, 32> = Console::new();
        console.register(&mut echo).unwrap();
        let out = run_script(&mut console, "help\nhelp echo\nhelp zz\n");
        assert_eq!(
            out,
            "help - list commands\n\
             echo - echo <words..> - print the arguments\n\
             echo <words..> - print the arguments\n\
             unknown command: zz\n"
        );
    }

    #[test]
    fn console_reports_bounded_input_errors() {
        let mut console: Console<'_, 1, 32> = Console::new();
        let mut out = String::new();
        for _ in 0..40 {
            console.feed(b'a', &mut out);
        }
        console.feed(b'\n', &mut out);
        console.feed_slice(b"1 2 3 4 5 6 7 8 9\n", &mut out);
        console.feed(0xC3, &mut out);
        console.feed(b'\n', &mut out);
        assert_eq!(
            out,
            "error: line too long\nerror: too many arguments\nerror: invalid input\n"
        );
    }

    #[test]
    fn empty_and_whitespace_lines_are_silent() {
        let mut console: Console<'_, 1, 32> = Console::new();
        let out = run_script(&mut console, "\n   \n\r\n");
        assert_eq!(out, "");
    }

    #[test]
    fn registry_rejects_overflow_and_duplicates() {
        let mut echo1 = Echo { calls: 0 };
        let mut echo2 = Echo { calls: 0 };
        let mut fail = Fail;
        let mut console: Console<'_, 1, 32> = Console::new();
        console.register(&mut echo1).unwrap();
        assert_eq!(console.register(&mut fail), Err(RegisterError::Full));
        let mut bigger: Console<'_, 2, 32> = Console::new();
        bigger.register(&mut echo2).unwrap();
        let mut echo3 = Echo { calls: 0 };
        assert_eq!(
            bigger.register(&mut echo3),
            Err(RegisterError::DuplicateName)
        );
    }
}
