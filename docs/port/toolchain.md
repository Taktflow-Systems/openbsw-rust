# Reproducible development toolchain

The checked-in `rust-toolchain.toml` is authoritative for Rust tooling.

| Tool | Policy |
|---|---|
| Rust compiler and Cargo | exactly 1.94.0 |
| Rust MSRV | 1.94.0 until a separately verified lower MSRV is adopted |
| Embedded target | `thumbv7em-none-eabihf` |
| Rust components | rustfmt, clippy, llvm-tools-preview |
| Python | 3.12 or newer |
| Python HIL packages | exact versions in `hil/requirements.txt` |
| C++ POSIX oracle | Ubuntu 24.04, CMake 3.28+, Ninja 1.11+, GCC/G++ 13+ |

## Clean-machine setup

On Windows with WSL2:

```powershell
rustup toolchain install 1.94.0 --profile minimal --component clippy,rustfmt,llvm-tools-preview --target thumbv7em-none-eabihf
python -m venv .venv
.venv\Scripts\python -m pip install --upgrade pip
.venv\Scripts\python -m pip install -r hil/requirements.txt
wsl --install -d Ubuntu-24.04
wsl -d Ubuntu-24.04 -- sudo apt-get update
wsl -d Ubuntu-24.04 -- sudo apt-get install -y build-essential cmake ninja-build git
```

On Linux, install the same native packages directly and use a Python virtual
environment. Then verify the setup:

```powershell
rustc --version
cargo --version
python -m pytest --version
cargo check --workspace --exclude bsw-bsp-stm32
cargo check -p bsw-bsp-stm32 --no-default-features --features stm32f413 --target thumbv7em-none-eabihf
cargo check -p bsw-bsp-stm32 --no-default-features --features stm32g474 --target thumbv7em-none-eabihf
```

The upstream oracle is independent of the production Cargo graph. Configure,
build, and test it with `tools/port/openbsw_oracle.ps1 -Action Test`.
