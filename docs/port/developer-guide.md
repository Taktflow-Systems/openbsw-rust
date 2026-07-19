# Developer quick start

Use Rust 1.94 and the locked workspace. On Windows:

```powershell
cargo test --workspace --exclude bsw-bsp-stm32 --all-features --locked
cargo run -p openbsw-reference-app -- --oracle
```

On Linux, run the same locked tests with `rust:1.94-bookworm`. The POSIX app
offers lifecycle, console, CAN, UDS/DoCAN, DoIP, storage/blob and simulated I/O
workflows. `docs/port/reference-app-parity.md` lists the stable observable rows.

For embedded development install `thumbv7em-none-eabihf`, then build exactly one
MCU feature:

```powershell
cargo build --locked --release -p bsw-bsp-stm32 --target thumbv7em-none-eabihf --no-default-features --features stm32f413 --example app_f413
cargo build --locked --release -p bsw-bsp-stm32 --target thumbv7em-none-eabihf --no-default-features --features stm32g474 --example app_g474
```

Run `tools/port/measure_resources.ps1 -Check` after code changes. Run the
commands in `docs/port/test-guide.md` before requesting review. Never place a
probe serial, COM port, login, token, personal path, or hostname in tracked
files; HIL receives them only through documented environment variables.
