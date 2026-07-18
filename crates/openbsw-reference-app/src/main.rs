use std::io::{self, BufRead};

use bsw_time::{Duration, Instant};
use openbsw_reference_app::scenario::{oracle_json, run_getting_started_workflows};
use openbsw_reference_app::{AppConfig, ReferenceApp};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|arg| arg == "--oracle") {
        print!("{}", oracle_json("rust"));
        return;
    }
    if args.iter().any(|arg| arg == "--workflows") {
        let results = run_getting_started_workflows();
        for result in &results {
            println!(
                "{}: {}",
                result.name,
                if result.passed { "PASS" } else { "FAIL" }
            );
        }
        if results.iter().any(|result| !result.passed) {
            std::process::exit(1);
        }
        return;
    }

    let mut app = ReferenceApp::new(AppConfig::default()).expect("configuration is valid");
    let mut now = Instant::from_nanos(0);
    app.start(now).expect("reference application starts");
    for line in app.logs() {
        println!("{line}");
    }
    for line in io::stdin().lock().lines() {
        let Ok(line) = line else {
            break;
        };
        now = now.wrapping_add(Duration::from_millis(1).expect("one millisecond fits"));
        if line == "quit" {
            break;
        }
        println!("{}", app.command(&line, now));
    }
    app.shutdown(now).expect("clean shutdown");
}
