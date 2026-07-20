# Upstream OpenBSW baseline

This port uses one immutable Eclipse OpenBSW revision as its behavioral and
structural source of truth:

- repository: <https://github.com/eclipse-openbsw/openbsw>
- commit: `be0029bbb79fe901048a24c2665f2ba854328734`
- commit date: 2026-06-02
- commit subject: `Add STM32 FDCAN transceiver adapter`
- license: Apache-2.0
- local, ignored oracle checkout: `target/oracle/openbsw`
- disposable native-WSL build mirror: `/tmp/openbsw-rust-oracle-be0029bbb79f`
  (Linux hosts build the ignored checkout in place via
  `tools/port/openbsw_oracle.sh`; no mirror is needed)
- previous baseline: `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77` (2026-03-12),
  re-pinned 2026-07-20 per `docs/port/upstream-repin-decision-2026-07-19.md`;
  release evidence captured against it remains valid historical record

The commit is intentionally a SHA rather than a branch or tag. Updating it is
a governed change: update this file, the parity manifest, oracle fixtures, and
baseline evidence in one review.

## Scope and source-of-truth order

The mandatory product scope is the generic BSW libraries, the POSIX reference
application as a behavioral oracle, and the STM32F413/STM32G474 Rust BSPs in
this repository. The upstream S32K148 platform is an optional expansion and
does not gate the mandatory completion percentage.

When sources disagree, use this order:

1. pinned upstream executable behavior and upstream tests;
2. pinned upstream public interfaces and implementation;
3. pinned upstream documentation;
4. current Rust behavior, until a parity decision records an intentional
   native replacement or exclusion.

The upstream library roots at the pinned revision are recorded row-by-row in
`parity-manifest.json`. The upstream reference application is
`executables/referenceApp`; its POSIX implementations are under
`platforms/posix` and `executables/referenceApp/platforms/posix`.

## Upstream capabilities in scope

The upstream README and module tree expose lifecycle management, asynchronous
execution, console commands and logging, classic CAN, ADC/PWM/GPIO,
UDS/DoCAN, Ethernet TCP/UDP, and non-volatile storage. The Rust port also has
project extensions (COM, time, E2E, blob storage, CAN FD) which are tracked
separately and cannot be used as evidence that an upstream row is complete.

## Licensing and notice obligations

The pinned tree is Apache-2.0 and includes an Eclipse `NOTICE.md`. A
distribution of derived or copied material must retain applicable copyright,
license, NOTICE, and trademark statements. Its notice identifies these
bundled third-party components and licenses:

| Component | Version | License |
|---|---:|---|
| Corrosion | 0.6 | MIT |
| Embedded Template Library | 20.48.1 | MIT |
| LwIP | 2.2.1 | BSD-3-Clause |
| GoogleTest | 1.12.1 | BSD-3-Clause |
| printf | 5.2.0 | MIT |
| FreeRTOS-Kernel | 10.6.2 | MIT |
| Eclipse ThreadX | 6.4.3 | MIT |
| CMSIS | 6.1.0 (at `libs/3rdparty/cmsis/LICENSE`) | Apache-2.0 |
| NXP S32K148 headers | 1.1a | BSD-3-Clause |
| ST STM32F4 Device Headers | 2.6.11 | Apache-2.0 |
| ST STM32G4 Device Headers | 1.2.6 | Apache-2.0 |
| CodeCoverage | pinned upstream copy | BSD-3-Clause |

Deltas versus the previous `ddbcf88` baseline (per
`docs/port/build-license-drift-review-2026-07-19.md`): ETL 20.46.2 ->
20.48.1 (MIT unchanged); CMSIS moved from the S32K platform tree to
`libs/3rdparty/cmsis/` (version and license unchanged); the ST STM32F4/G4
device-header rows are new with upstream's STM32 platform tree. No license
terms changed. The port's shipped Rust artifacts bundle none of these
components, so its own NOTICE obligations are unchanged.

This inventory is a baseline, not the final distribution audit. Any source
copied from upstream must preserve its SPDX/copyright header. Rust-native
rewrites should cite the upstream module in documentation without presenting
Eclipse names or marks as endorsement.

## Reproducing the checkout

Run `tools/port/openbsw_oracle.ps1 -Action Checkout` (Windows/WSL) or
`tools/port/openbsw_oracle.sh Checkout` (Linux). Both refuse to build a
checkout whose `HEAD` differs from the pinned SHA. Generated upstream build
products remain in the ignored checkout (or the disposable WSL mirror on
Windows) and never enter the Rust workspace dependency graph. The Windows
mirror avoids the severe C++ compilation penalty of a Windows-mounted source
tree and is always checked out at the same pinned commit before a build.

The pinned tree's minimum C++ standard is C++17, its build layout is
Bazel-primary with CMake presets retained, and the middleware libraries use
build-time code generation (`jinja2cpp.py`, wired through CMake
`add_custom_command`; requires python3 with `jinja2` and `yaml`). The oracle
scripts use the retained CMake presets `tests-posix-debug` and
`posix-freertos`; Bazel is not required.
