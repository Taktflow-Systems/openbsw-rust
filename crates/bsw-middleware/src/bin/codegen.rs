use std::env;
use std::fs;
use std::path::PathBuf;

use bsw_middleware::codegen::{generate, parse_model};

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let mut args = env::args().skip(1);
    let input = PathBuf::from(
        args.next()
            .ok_or("usage: bsw-middleware-codegen INPUT OUTPUT [--check]")?,
    );
    let output = PathBuf::from(
        args.next()
            .ok_or("usage: bsw-middleware-codegen INPUT OUTPUT [--check]")?,
    );
    let check = args.next().is_some_and(|arg| arg == "--check");
    if args.next().is_some() {
        return Err("unexpected extra argument".to_owned());
    }
    let source =
        fs::read_to_string(&input).map_err(|error| format!("{}: {error}", input.display()))?;
    let model = parse_model(&source)
        .map_err(|error| format!("{}:{}: {:?}", input.display(), error.line, error.kind))?;
    let generated = generate(&model);
    if check {
        let existing = fs::read_to_string(&output)
            .map_err(|error| format!("{}: {error}", output.display()))?;
        if existing != generated {
            return Err(format!(
                "generated middleware output is stale: {}",
                output.display()
            ));
        }
    } else {
        fs::write(&output, generated).map_err(|error| format!("{}: {error}", output.display()))?;
    }
    Ok(())
}
