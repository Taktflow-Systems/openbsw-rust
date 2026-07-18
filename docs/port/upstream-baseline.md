# Upstream OpenBSW baseline

This port uses one immutable Eclipse OpenBSW revision as its behavioral and
structural source of truth:

- repository: <https://github.com/eclipse-openbsw/openbsw>
- commit: `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`
- commit date: 2026-03-12
- commit subject: `RTOS-1794 fix CT-P1 clang-tidy violations in UDS production paths`
- license: Apache-2.0
- local, ignored oracle checkout: `target/oracle/openbsw`
- disposable native-WSL build mirror: `/tmp/openbsw-rust-oracle-ddbcf88a62df`

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
| Embedded Template Library | 20.46.2 | MIT |
| LwIP | 2.2.1 | BSD-3-Clause |
| GoogleTest | 1.12.1 | BSD-3-Clause |
| printf | 5.2.0 | MIT |
| FreeRTOS-Kernel | 10.6.2 | MIT |
| Eclipse ThreadX | 6.4.3 | MIT |
| CMSIS | 6.1.0 | Apache-2.0 |
| NXP S32K148 headers | 1.1a | BSD-3-Clause |
| CodeCoverage | pinned upstream copy | BSD-3-Clause |

This inventory is a baseline, not the final distribution audit. Any source
copied from upstream must preserve its SPDX/copyright header. Rust-native
rewrites should cite the upstream module in documentation without presenting
Eclipse names or marks as endorsement.

## Reproducing the checkout

Run `tools/port/openbsw_oracle.ps1 -Action Checkout`. The script refuses to
build a checkout whose `HEAD` differs from the pinned SHA. Generated upstream
build products remain in the disposable WSL mirror and never enter the Rust
workspace dependency graph. The mirror avoids the severe C++ compilation
penalty of a Windows-mounted source tree and is always checked out at the same
pinned commit before a build.
