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
python tools/port/check_tms570_startup.py
python tools/port/check_tms570_ram_marker.py
python tools/port/check_tms570_exception_probe.py
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

Run all 14 fuzz targets in a Linux `rust:1.94-bookworm` container with
`bash tools/port/run_fuzz_linux.sh`. The script pins the nightly toolchain and
requires 100 libFuzzer runs for every target. `doip_entity` covers the
portable embedded DoIP entity over the deterministic fake stack boundary
(optional embedded-DoIP tranche; see `embedded-doip-2026-07-19.md`).

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

## Optional TMS570 foundation checks

Run the target-independent LC4357 register/state models and the shared
embedded DoIP boundary with:

```powershell
cargo test -p bsw-bsp-tms570 --features launchxl2-570lc43 --locked
cargo clippy -p bsw-bsp-tms570 --all-targets `
  --features launchxl2-570lc43 --locked -- -D warnings
cargo check -p bsw-doip --no-default-features --locked
cargo test -p bsw-doip --all-features --locked
cargo clippy -p bsw-ethernet -p bsw-doip --all-targets `
  --all-features --locked -- -D warnings
```

Cross-build the library, minimal link probe, startup inspection probe, and
RAM-only roles with the commands in `release-build.md`. The 42 BSP model tests
additionally cover
the exact LAUNCHXL2-570LC43 oscillator, MII/DP83630 path, PHY strap/address,
reference clock, active-low reset and quiescent power-down state, alongside
the device geometry, startup ordering, clock ratios, all three PLL advisory
failure signatures, five-attempt fail-closed behavior, DCC windows, MDIO,
CPPI invariants, the exact RTI prescaler, 32-bit counter wrap extension,
critical-section nesting, saved mask classification, unbalanced release, and
the 128-channel VIM registration table, reserved/duplicate rejection,
IRQ/FIQ class selection, index decoding, enable/disable, and bounded default
capture. Five retained-exception tests cover all A32 LR corrections, banked
modes, fault-address validity, exact target-order CRC, and fail-closed corrupt,
stale, recursive, unknown, Thumb, little-endian, and wrong-mode records.

The clock probe is inspected separately from the marker. It is bounded to
SRAM, rejects stack/call/literal-pool behavior and unreviewed store offsets,
and has no data or BSS:

```powershell
python tools/port/check_tms570_clock_probe.py
```

Two physical controlled-reset runs of the same inspected clock artifact passed the
five-attempt SSWF021#45 policy on attempt one, measured PLL1 inside the declared
300-MHz DCC window, and read back GCLK/HCLK/VCLK at 300/150/75 MHz.

The RTI role is inspected separately and loaded into a disjoint 256-byte SRAM
window only after the clock role passes:

```powershell
python tools/port/check_tms570_rti_probe.py
```

It rejects stack, calls, literal pools, and every store outside RTI counter 0.
Two current-artifact clean-start runs passed exact IRQ/FIQ mask restoration,
configured CPUC0=74 from 75-MHz RTICLK, crossed an in-firmware 100,000-us
threshold, and observed 361,895/351,089 RTI microseconds over coarse debugger
intervals of 367/339 ms. The broad host interval is corroboration, not a
precision calibration; DCC provides the independent oscillator reference.
Counter 1 and DWWD were untouched.

The VIM role is independently linked into `[0x08075000, 0x08075c00)` and
inspected with:

```powershell
python tools/port/check_tms570_vim_probe.py
```

The checker proves BE32 A32, separate 256-byte IRQ/FIQ stack reservations,
channel-2 vector-RAM registration, exact VIM class/mask operations, two
architectural exception returns, reviewed RTI/ESM/VIM stores, and a bounded
1,152-byte text segment with no data or BSS. Two controlled-reset runs of the
same inspected ELF passed fresh RTI compare-0 IRQ and FIQ delivery exactly
once each with IRQINDEX/FIQINDEX value 3. The role handled revision-B
DEVICE#56 by accepting and clearing only Group-2 channel 2 in both live and
shadow status, then proved every unrelated ESM status word unchanged. Fixed
flash IRQ/FIQ trampolines were read and validated but never modified.

The retained-exception role is independently linked into its SRAM code,
record, and stack windows and inspected with:

```powershell
python tools/port/check_tms570_exception_probe.py
```

The checker proves BE32 A32, distinct Undefined/SVC/Prefetch/Data entries,
full entry frames, banked source SP/LR transfer, exact LR corrections, six
fault-register reads, commit-last CRC publication, read-only fault injection,
and exactly one reviewed reset MMIO store. The private runner catches each
fixed flash vector after architectural entry, redirects only PC to the RAM
handler, validates the committed record before reset, then validates the same
record at reset vector zero. Two runs for each class passed (8/8). See
`tms570-exception-capture.md` for the proof boundary.

The same inspected 12-byte, no-stack SRAM marker passed two physical
clean-start debugger-load runs on the confirmed LAUNCHXL2-570LC43. The marker,
clock, staged RTI, staged VIM, and staged retained-exception roles are the only
physical TMS570 execution passes in this checkout.
Do not relabel the
host fake, POSIX adapter, flash-mapped startup inspection probe, or passive
Ethernet observation as HIL. The operator confirmed a bare board; subsequent
physical work proceeds in the bounded order in
`docs/plans/plan-tms570lc4357-platform.md`.
