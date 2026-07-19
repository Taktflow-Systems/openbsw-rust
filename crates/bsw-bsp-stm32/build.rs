use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    // Copy the chip-specific memory.x to OUT_DIR so cortex-m-rt's link.x can find it.
    if env::var("CARGO_FEATURE_STM32F413").is_ok() {
        fs::copy("memory_f413.x", out_dir.join("memory.x")).unwrap();
    } else if env::var("CARGO_FEATURE_STM32G474").is_ok() {
        fs::copy("memory_g474.x", out_dir.join("memory.x")).unwrap();
    }

    fs::copy("rom_crc.x", out_dir.join("rom_crc.x")).unwrap();

    println!("cargo:rustc-link-search={}", out_dir.display());
    if env::var("TARGET").is_ok_and(|target| target.starts_with("thumbv7em")) {
        println!("cargo:rustc-link-arg=-Trom_crc.x");
    }
    println!("cargo:rerun-if-changed=memory_f413.x");
    println!("cargo:rerun-if-changed=memory_g474.x");
    println!("cargo:rerun-if-changed=rom_crc.x");
    println!("cargo:rerun-if-changed=build.rs");
}
