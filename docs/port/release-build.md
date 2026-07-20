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
