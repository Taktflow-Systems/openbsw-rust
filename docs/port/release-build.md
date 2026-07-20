# Reproducible release build

Release baseline: Rust 1.94, locked `Cargo.lock`, pinned upstream commit
`be0029bbb79fe901048a24c2665f2ba854328734` (re-pinned 2026-07-20; the
completed 2026-07-18 mandatory release was built against the previous pin
`ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77` and its evidence is unchanged),
LTO, one codegen unit,
`opt-level=s`, aborting panic and overflow checks. The Rust workspace may be
dirty during tranche review. `create_clean_build_roots.ps1` reconstructs two
immutable roots from the recorded base revision plus a binary source patch and
sorted untracked-source manifest. Its private manifest records the base,
locked inputs, complete source-file inventory, and deterministic source-bundle
identity without altering Git history.

Build POSIX twice at the stable container path `/work` with
`rust:1.94-bookworm`. Build each STM32 example twice with separate
`CARGO_TARGET_DIR` values and the exact commands in `developer-guide.md`.
`build_posix_reproducible.ps1` builds both roots in the pinned container with
the identical `/work` source path. `build_release_candidate.ps1` produces both
MCU example sets and link maps; `assemble_hil_artifacts.ps1` independently
embeds each release ROM CRC, compares all 14 HIL artifacts byte-for-byte,
records size output, and selects the exact compared set. Enforce the stronger
resource report with `measure_resources.ps1 -Check`.

The release manifest, hashes, size/map summaries, SBOM, license/advisory result,
test counts and exact-artifact HIL result are indexed by
`docs/test-evidence/samples/g12-i09-final-matrix.json`.

The publishable final matrix uses private artifact identity references. Exact
firmware/build digests, source-bundle digest, binaries, maps, and machine-local
paths remain only in ignored `target/private-evidence/` CI artifacts. The
matrix records whether A/B comparison passed; resource sizes are publishable
in `resource-baseline-2026-07-18.md` because they do not identify a private
firmware image.

## Optional TMS570 toolchain probe

TMS570LC4357 is BE32 Armv7-R. Rust exposes `armebv7r-none-eabihf` as a tier-3
target, so the optional build uses the pinned `nightly-2026-03-14` toolchain
with `rust-src` to build `core`. GNU Arm linking must include `-mbig-endian`
and `-mbe32`; the installed linker was independently checked with a minimal
Arm assembly image.

The current Rust link probe is deliberately non-bootable and must never be
loaded into hardware:

```powershell
cargo +nightly-2026-03-14 rustc -Z build-std=core `
  --target armebv7r-none-eabihf `
  -p bsw-bsp-tms570 --features launchxl2-570lc43 `
  --example link_probe --locked -- `
  -C linker=arm-none-eabi-gcc `
  -C link-arg=-mcpu=cortex-r5 -C link-arg=-marm `
  -C link-arg=-mfloat-abi=hard -C link-arg=-mfpu=vfpv3-d16 `
  -C link-arg=-mbig-endian -C link-arg=-mbe32 `
  -C link-arg=-nostartfiles -C link-arg=-nostdlib `
  -C link-arg=-Tcrates/bsw-bsp-tms570/linker/tms570lc4357-link-probe.ld
```

`arm-none-eabi-readelf` must report ELF32, big endian, Arm, EABI5,
hard-float, Armv7 Realtime profile, and entry `0x08000000`.
`arm-none-eabi-objdump` must show A32 instructions. This closes only the target
and linker probe; it is not startup, BSP, board, Ethernet, or DoIP evidence.

The flash-mapped startup inspection probe adds the exact reset/vector, bounded
SRAM/VIM ECC initialization, banked-mode stacks, hard-float enable, Rust memory
initialization, and default VIM table. It is also compile/disassembly-only and
must not be loaded or flashed:

```powershell
cargo +nightly-2026-03-14 rustc -Z build-std=core `
  --target armebv7r-none-eabihf `
  -p bsw-bsp-tms570 --features launchxl2-570lc43 `
  --example startup_probe --locked -- `
  -C linker=arm-none-eabi-gcc `
  -C link-arg=-mcpu=cortex-r5 -C link-arg=-marm `
  -C link-arg=-mfloat-abi=hard -C link-arg=-mfpu=vfpv3-d16 `
  -C link-arg=-mbig-endian -C link-arg=-mbe32 `
  -C link-arg=-nostartfiles -C link-arg=-nostdlib `
  -C link-arg=-Tcrates/bsw-bsp-tms570/linker/tms570lc4357-startup-probe.ld
```

The linker asserts the 32-byte vector table at zero, 512 KiB SRAM limit, all
mode-stack boundaries, and flash load range. `readelf` must show entry `0x20`,
`.vectors` at `0x00000000`, `.stacks` at `0x0807de00` with size `0x2200`, and
big-endian EABI5 hard-float. `objdump` must prove the IRQ/FIQ VIM loads, no SP
use before `MINIDONE`, every mode switch/SP load, FPU enable, memory loops, and
the 128-word VIM initialization. The first physical artifact remains a
separately reviewed RAM role; this flash layout is not flash authorization.

The reviewed physical role is a register-only SRAM marker. It has no stack,
data, BSS, MMIO, or flash-mapped segment and is constrained to the 256-byte
window starting at `0x08078000`:

```powershell
cargo +nightly-2026-03-14 rustc -Z build-std=core `
  --target armebv7r-none-eabihf `
  -p bsw-bsp-tms570 --features launchxl2-570lc43 `
  --example ram_marker --locked -- `
  -C linker=arm-none-eabi-gcc `
  -C link-arg=-mcpu=cortex-r5 -C link-arg=-marm `
  -C link-arg=-mfloat-abi=hard -C link-arg=-mfpu=vfpv3-d16 `
  -C link-arg=-mbig-endian -C link-arg=-mbe32 `
  -C link-arg=-nostartfiles -C link-arg=-nostdlib `
  -C link-arg=-Tcrates/bsw-bsp-tms570/linker/tms570lc4357-ram-marker.ld
python tools/port/check_tms570_ram_marker.py
```

The checker requires one 12-byte executable load segment, entry and symbol at
`0x08078000`, exact `movw`/`movt` marker construction followed by a self-loop,
and rejects stack, memory, call, or return instructions. The same inspected
artifact passed two physical clean-start RAM-load runs. This is evidence only
for debugger load and instruction execution; it is not flash, startup-vector,
clock, peripheral, Ethernet, or DoIP evidence.

The next physical role qualifies PLL1 and the synchronous clock dividers from
SRAM. It uses the same target/link flags with `--example clock_probe` and:

```powershell
-C link-arg=-Tcrates/bsw-bsp-tms570/linker/tms570lc4357-clock-probe.ld
python tools/port/check_tms570_clock_probe.py
```

The linker confines it to `[0x08077000, 0x08078000)` with no data or BSS. The
checker rejects stack, call, return, literal-pool, multi-register, and
unreviewed MMIO-store behavior. Two controlled-reset runs of the same inspected
artifact passed the bounded SSWF021#45 sequence, DCC qualification, and exact
300/150/75-MHz readback. It performs no flash, pinmux, GPIO, PHY, EMAC, CAN,
watchdog, or security action.

The RTI/critical-section role is staged only after the unchanged clock probe
has passed. Build it with the same target/link flags, `--example rti_probe`,
and:

```powershell
-C link-arg=-Tcrates/bsw-bsp-tms570/linker/tms570lc4357-rti-probe.ld
python tools/port/check_tms570_rti_probe.py
```

The linker confines it to `[0x08076000, 0x08076100)`. The checker requires
BE32 A32, CPSR IRQ/FIQ mask/restore, CPUC0=74, a 100,000-tick bound, coherent
FRC0-then-UC0 reads, and stores only to RTI counter-0 registers. It rejects
stack, calls, returns, literal pools, and multi-register operations. Two
current-artifact controlled-reset runs passed; this does not authorize flash
or claim VIM, watchdog, Ethernet, or DoIP operation.

The VIM role is staged only after the unchanged clock probe passes. Build it
with the same target/link flags, `--example vim_probe`, and:

```powershell
-C link-arg=-Tcrates/bsw-bsp-tms570/linker/tms570lc4357-vim-probe.ld
python tools/port/check_tms570_vim_probe.py
```

The linker confines executable content to `[0x08075000, 0x08075c00)`, reserves
56 private state bytes and separate 256-byte IRQ/FIQ stacks, and rejects data
or BSS. The checker requires BE32 A32, channel-2 vector registration, exact
IRQ/FIQ classification, two exception returns, DEVICE#56 live/shadow
acknowledgement, reviewed MMIO stores, and the bounded 1,152-byte text segment.
Two controlled-reset runs of the same inspected artifact passed one RTI
compare-0 IRQ and FIQ each with index 3. It reads but never writes the resident
flash vector trampolines and does not authorize flash or claim Ethernet/DoIP.

The retained-exception role is built with the same target/link flags,
`--example exception_probe`, and:

```powershell
-C link-arg=-Tcrates/bsw-bsp-tms570/linker/tms570lc4357-exception-probe.ld
python tools/port/check_tms570_exception_probe.py
```

The linker confines 1,836 executable bytes to `[0x08073000,0x08074000)`, fixes
a 160-byte record at `0x08074000`, and reserves separate Undefined, Abort,
Supervisor, and System stacks through `0x08074800`. The checker requires the
four entries, banked context, exact A32 LR corrections, CP15 fault registers,
CRC/commit ordering, read-only bounded injections, and the sole reviewed
`SYS1.SYSECR` reset store. Eight controlled-reset runs passed: two each for
undefined, SVC, prefetch abort, and data abort. Fixed flash vectors were only
read/caught; the debugger redirected PC after architectural entry and the
record carries that provenance. The image has no flash load segment and is not
flash authorization.

The flash startup inspection probe now also contains the same terminal entries
and a linker-fixed 160-byte production record at the start of SRAM bank 7. Its
cold path initializes all banks; its warm path excludes bank 7, preserves only
the record, and regenerates ECC for every other bank-7 doubleword before stack
use. This remains compile/map/disassembly evidence, not a physical startup
claim.
