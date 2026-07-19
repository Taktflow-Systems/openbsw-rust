# Verification guide

The release gate is fail-closed. Core commands are:

```powershell
cargo fmt --all -- --check
cargo test --workspace --exclude bsw-bsp-stm32 --all-features --locked
cargo clippy --workspace --exclude bsw-bsp-stm32 --all-targets --all-features --locked -- -D warnings
python tools/port/check_no_std.py
python tools/port/check_dependencies.py
python tools/port/check_features.py
python tools/port/check_stm32_examples.py
powershell -File tools/port/measure_resources.ps1 -Check
python tools/port/inventory_unsafe.py --check
python tools/port/generate_status.py --check docs/port/status.md
python tools/port/generate_sbom.py --output docs/port/sbom.cdx.json --check
python tools/port/validate_evidence.py
python tools/port/check_privacy.py
cargo deny check licenses advisories bans sources
```

Cross-check each MCU library and every declared example with exactly one
feature. Linux additionally runs the POSIX SocketCAN/vcan and DoIP integration,
oracle comparison, fuzz smoke, and the pinned container test. Miri uses the
pinned nightly named in `safety-verification-2026-07-18.md`.

Run all 13 fuzz targets in a Linux `rust:1.94-bookworm` container with
`bash tools/port/run_fuzz_linux.sh`. The script pins the nightly toolchain and
requires 100 libFuzzer runs for every target.

Authoritative HIL is never skip-capable. Run two deterministic application
smokes, the destructive linker-bounded storage role, the queue-overflow role,
the TX-isolated real-controller bus-off role, ten resets, and the ten-minute
bounded soak. A pass requires the old or new journal value after every cut,
nonzero overflow accounting followed by a valid request, observed BusOff then
Active, ten boot markers, at least 64 burst responses plus final recovery, a
99.9% response ratio at 10 Hz for ten minutes, and final recovery after soak.
`hil/stress_campaign.py` applies those explicit thresholds to an artifact path
supplied by the release build.

For final acceptance, first reconstruct two clean roots with
`create_clean_build_roots.ps1`, build both with `build_release_candidate.ps1`,
build POSIX twice at `/work` with `build_posix_reproducible.ps1`, then compare
and select the MCU artifacts with `assemble_hil_artifacts.ps1`. Pass that
immutable `selected` directory to `run_physical_matrix.ps1`; do not rebuild a
substitute between regression, HIL, and soak. Exact identities and bench-local
selectors stay under ignored `target/private-evidence/`.
