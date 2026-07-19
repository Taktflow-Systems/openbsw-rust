# Final mandatory acceptance review - 2026-07-19

Release baseline: Eclipse openBSW
`ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`.

| Section 11 criterion | Result | Evidence |
|---:|---|---|
| 1 | pass: 37/37 mandatory rows are `done` or `native replacement` | `status.md`, `parity-manifest.json` |
| 2 | pass: each native replacement has a strategy and concrete crate/test evidence | `parity-manifest.json`, `reference-app-parity.md` |
| 3 | pass: production Cargo graphs contain no openBSW C++ implementation | `production-reference.md`, `dependency-policy.json` |
| 4 | pass: portable production crates are allocation-free `no_std` | `developer-guide.md`, final matrix no-std gate |
| 5 | pass: formatting, clippy, host/Linux tests, fuzz, and both target checks are green | `../test-evidence/samples/g12-i09-final-matrix.json` |
| 6 | pass: all 392 production unsafe rows are fingerprinted and approved, and affected abstractions passed 337 Miri tests | `unsafe-inventory.md`, `unsafe-review-approvals.json`, final matrix |
| 7 | pass: the POSIX reference application covers the required application surfaces | `reference-app-parity.md`, final matrix workflows |
| 8 | pass: seven differential fixture pairs match the pinned oracle | `oracle-fixtures/`, final matrix |
| 9 | pass: both MCU applications compose `bsw-reference-core` and shared protocol crates | `production-reference.md`, `stm32-parity.md` |
| 10 | pass: both boards passed deterministic smoke, recovery, stress, reset, and ten-minute soak gates | `stm32-parity.md`, final matrix |
| 11 | pass: Ethernet, MPU, ECC, watchdog, and measurement limits are explicit | `safety-mechanisms-2026-07-18.md`, `board-setup.md` |
| 12 | pass: flash, BSS, buffers, stack paths, queues, timing, and code size are below enforced limits | `resource-baseline-2026-07-18.md`, `resource-limits.json` |
| 13 | pass: production roles are thin; mandatory tests have no unexplained failure or flake | `reference-app-parity.md`, `stm32-parity.md`, final matrix |
| 14 | pass: license/NOTICE, SBOM, audit, privacy, reproducibility, and release evidence gates pass | `../../LICENSE`, `../../NOTICE`, `sbom.cdx.json`, `release-build.md`, final matrix |
| 15 | pass: architecture, developer, board, test, and release instructions cover checkout to all three applications | `../architecture/production-reference.md`, `developer-guide.md`, `board-setup.md`, `test-guide.md`, `release-build.md` |

No severity-1 or severity-2 defect remains open. Three documentation examples
remain intentionally ignored; no executable mandatory test is skipped. The
transitive archived `bare-metal 0.2.5` advisory is explicitly accepted because
it is pulled by `cortex-m 0.7.7`, reports no vulnerability, and has no safe
upgrade. This is a maintenance item, not a hidden audit failure.

The H01-H09 mechanisms are non-certification support mechanisms. This review
does not claim ISO 26262 certification, AUTOSAR certification, quantitative
diagnostic coverage, on-board Ethernet on either reference board, F413 ECC, or
G474 RAM ECC diagnostics.
