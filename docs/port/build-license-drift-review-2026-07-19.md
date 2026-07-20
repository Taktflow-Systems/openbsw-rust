# Build/toolchain and license/NOTICE drift review (2026-07-19)

Follow-up package U05 from `docs/port/upstream-drift-2026-07-18.md`. This memo
surveys the build/toolchain and ETL portions of upstream drift and performs the
license/NOTICE delta review between the pinned oracle revision and the drift
tip. It is informational only: no SBOM, NOTICE, dependency-policy, parity, or
evidence file is changed by this review, and the reasons are evidenced below.

- pinned oracle: `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77` (2026-03-12)
- drift tip: `be0029bb` (bare read-only mirror `target/drift/openbsw-drift.git`)
- drift-exclusive commit inventory: `target/drift/analysis/area-map.tsv`
  (166 commits), changed-path inventory: `target/drift/analysis/paths-all.txt`
- baseline licensing table: `docs/port/upstream-baseline.md`

## 1. Build/toolchain drift survey

The area map assigns 46 drift commits to the `build/toolchain` bucket.
Grouping by subject:

| Group | Commits | Representative subjects |
|---|---:|---|
| Bazel migration / BUILD files | 13 | `68aa1fd1` BUILD.bazel for libs/bsw; `a32e0f0e`, `425b1f5a` common/bsp/configuration BUILD files; `e39372a3`, `5cb140d3` printf BUILD; `b3cd5e77` Bazel CI + buildifier; `cb12f1d3` buildifier in treefmt; `d611c0f3` lockfile regen; `0f3bf40c`, `c45100ba` ETL rim ignore lists; `0fac881c` Bazel formatting; `b4b49806` bazel.yml; `58ef9c2e` gcc arm-none-eabi toolchain POC |
| clang-tidy infrastructure | 6 | `2371385d` reintroduce gating; `026a7630`, `4f3a4bf9`, `e0cb9558` CI scope; `76045328` s32k build-graph fix; `49933365` CT-P1 fixes in cpp2ethernet |
| Copyright-header tooling | 6 | `6e65ea94` cr_checker + CI workflow (`.github/workflows/copyright.yml`); `c21dcc3a` automated Eclipse headers on all sources; `02b9a4b1` docs + treefmt integration; `b352492a`, `b32ed8e5` config fixes; `1211c9c8` formatting guidelines (pragma once, header) |
| CI / docker | 5 | `232c1bba` cargo/rustup homes for runtime UID; `dfd96950` compose proxy vars; `b703c4a5` minimal cache mode; `a3749e83` bind-mount permissions; `c73614fc` cache dir permissions |
| ETL / NOTICE updates | 3 | `823fc846` NOTICE for ETL 20.47.1; `17a5d461` NOTICE for ETL 20.48.1; `1844a9d4` ETL formatting for example/logger time |
| Toolchain version bumps | 2 | `8bcfb3dc` upstream default Rust 1.96.0; `e45eee34` minimum C++17 |
| Codegen / build infra | 2 | `c4401a13` middleware build-time generation; `24393a44` venv wrappers for blob regeneration |
| Third-party RIM metadata | 1 | `0afc5daa` RIM metadata for CMSIS |
| Formatting / repo config misc | 8 | `5f33d338`, `adc0309a`, `9f61d48b`, `339f0dab` formatting; `c2f709ba` .gitattributes; `f240b04f` .gitignore; `5df5c25b` artifact-analysis doc removal; `d0bbffaf` review follow-up |

Total: 46 (matches the bucket count).

Bazel migration work also landed in functional buckets: `9e4fbd4b` (ThreadX),
`6c074682` and `87b8ad1d` (FreeRTOS), `a63bab57` (async config select) in
async/executors; `57d27137` (transport/uds) in UDS; `c9ec30bc` (docan, doip,
logger_integration) in DoCAN; `415a05cf` (lwip) and several target-cleanup
commits in other; `8857f6b9` in reference application. A strict subject match
on bazel/BUILD/buildifier/clang-tidy yields 23 commits (19 build/toolchain,
3 async/executors, 1 UDS); adding the two migration subjects that omit the
keyword (`415a05cf`, `c9ec30bc`) reconciles the recorded signal of 25
bazel/clang-tidy subjects. At the tip the upstream root gains `.bazelrc`,
`.bazelversion`, `.bazelignore`, `BUILD.bazel`, `MODULE.bazel`,
`MODULE.bazel.lock`, `bazel/`, and `bazel_migration/`; CMake files remain.

### ETL drift

36 drift subjects mention ETL or estd (18 DoIP, 12 other, 5 build/toolchain,
1 CAN/STM32). Content:

- Version path: 20.46.2 (pin) -> 20.47.1 (`8e4ba7d5` + NOTICE `823fc846`) ->
  20.48.0 (`0aa77ad5`) -> 20.48.1 (`23d0da68` + NOTICE `17a5d461`). Verified
  from `libs/3rdparty/etl/version.txt`: pin `20.46.2`, tip `20.48.1`.
- Interface adaptation to new ETL releases: `d7bf47c3`, `b3ab7a80`,
  `830b9c5e` (STM32 ETL clock).
- Broader ETL adoption replacing estd/local code: DoIP estd-to-ETL series
  (18 commits, e.g. `8d18b1a1` slice-to-span through `baa9589d`), `eb956887`
  remove estd containers, `c1d18f8d` CRC via ETL, `db24855a`
  `etl/queue_spsc_atomic.h`, `0514bedc` fully qualified `::etl`, `96ba360a`,
  `1b2f8d0e` ETL error/assert usage, `4980b7c3` ETL print.

### Does any of this affect how this Rust workspace builds, tests, or ships?

No. Verification performed in this workspace:

- `rg` over all `build.rs`, all `Cargo.toml` manifests, `tools/`, and `*.toml`
  for bazel, BUILD.bazel, buildifier, cmake, CMakeLists, corrosion,
  clang-tidy, treefmt: the only hits are the CMake preset invocations inside
  `tools/port/openbsw_oracle.ps1`, which build the pinned oracle checkout in
  its disposable mirror. Per `docs/port/upstream-baseline.md`, oracle build
  products never enter the Rust workspace dependency graph, and the script
  refuses any HEAD other than the pinned SHA, so upstream's Bazel migration
  cannot leak in.
- `crates/bsw-bsp-stm32/build.rs` (the only build script in the workspace)
  contains no reference to any upstream build system.
- The Rust workspace consumes no upstream C++ dependency: ETL, LwIP,
  FreeRTOS, ThreadX, CMSIS, GoogleTest, printf, Corrosion, and the NXP
  headers appear nowhere in `docs/port/sbom.cdx.json` (43 components, every
  one a `pkg:cargo/` purl).
- Upstream's Rust-facing commits (`8bcfb3dc` default Rust 1.96.0, `1cf01d02`
  vendored Rust deps under `libs/3rdparty/cargo-vendor/`, `232c1bba` docker
  cargo homes, `b574834a`/`da6b3285` upstream Rust crates) configure the
  upstream repository's own Rust integration and are not consumed here; this
  workspace pins its own toolchain and lockfile.

## 2. License/NOTICE delta (pin -> tip)

`git diff ddbcf88a62df be0029bb -- NOTICE.md LICENSE` in the bare mirror
shows exactly one changed file: `NOTICE.md` (+4/-2). The root `LICENSE`
(Apache-2.0) is byte-identical. The NOTICE delta:

1. ETL row: version `20.46.2` -> `20.48.1` (license stays MIT, path
   unchanged).
2. CMSIS row: path moved from
   `platforms/s32k1xx/bsp/bspMcu/include/3rdparty/cmsis/LICENSE` to
   `libs/3rdparty/cmsis/LICENSE` (`67e94d82` extraction + `0afc5daa` RIM
   metadata); version 6.1.0 and Apache-2.0 unchanged.
3. Two new bundled components: ST STM32F4 Device Headers 2.6.11 and ST
   STM32G4 Device Headers 1.2.6, both Apache-2.0, at
   `platforms/stm32/bsp/bspMcu/include/3rdparty/st/LICENSE` (upstream's new
   STM32 platform work).

Per-file check of every bundled license text between pin and tip: root
`LICENSE`, `libs/3rdparty/etl/LICENSE`, `libs/3rdparty/corrosion/LICENSE`,
`libs/3rdparty/lwip/COPYING`, `libs/3rdparty/googletest/LICENSE`,
`libs/3rdparty/printf/LICENSE`, `libs/3rdparty/freertos/LICENSE.md`,
`libs/3rdparty/threadx/LICENSE.md` (all three ThreadX copies), and
`cmake/modules/CodeCoverage.cmake` are all unchanged. No license terms
changed anywhere; the only substantive deltas are one version bump (ETL) and
two additive components (ST headers).

Automated copyright-header commit: `c21dcc3a` (2026-05-27, "Apply Eclipse
copyright headers to all source files (automated)"), backed by new
`tools/cr_checker/` tooling (`6e65ea94`) and a `copyright.yml` CI gate. This
rewrites header comments on upstream's own files (BMW AG copyright,
`SPDX-License-Identifier: Apache-2.0`); it changes no license. Other
license-relevant paths in `paths-all.txt`: `libs/3rdparty/cargo-vendor/{log,paste}/LICENSE-{MIT,APACHE}`
(upstream's vendored Rust deps, dual MIT/Apache-2.0) and `tools/blob/NOTICE.md`
(upstream-authored tooling, Apache-2.0). Both are upstream-internal.

### Bundled-component table, pin vs drift tip

| Component | Pin (ddbcf88) | Tip (be0029b) | License pin -> tip | Delta |
|---|---:|---:|---|---|
| Corrosion | 0.6 | 0.6 | MIT -> MIT | none |
| Embedded Template Library | 20.46.2 | 20.48.1 | MIT -> MIT | version only |
| LwIP | 2.2.1 | 2.2.1 | BSD-3-Clause | none |
| GoogleTest | 1.12.1 | 1.12.1 | BSD-3-Clause | none |
| printf | 5.2.0 | 5.2.0 | MIT | none |
| FreeRTOS-Kernel | 10.6.2 | 10.6.2 | MIT | none |
| Eclipse ThreadX | 6.4.3 | 6.4.3 | MIT | none |
| CMSIS | 6.1.0 | 6.1.0 | Apache-2.0 | path move only |
| NXP S32K148 headers | 1.1a | 1.1a | BSD-3-Clause | none |
| CodeCoverage | pinned copy | same | BSD-3-Clause | none |
| ST STM32F4 Device Headers | not bundled | 2.6.11 | n/a -> Apache-2.0 | new at tip |
| ST STM32G4 Device Headers | not bundled | 1.2.6 | n/a -> Apache-2.0 | new at tip |

## 3. Impact on this port's shipped artifacts

None. The shipped Rust artifacts contain no upstream C++ code and no bundled
upstream component: `docs/port/sbom.cdx.json` lists 43 components, all with
`pkg:cargo/` purls generated from `Cargo.lock` by
`tools/port/generate_sbom.py`; a name search for
etl/lwip/freertos/threadx/cmsis/googletest/corrosion/printf/nxp over the SBOM
returns nothing. The port's NOTICE obligations therefore derive from its own
Apache-2.0 workspace and its cargo dependency graph, neither of which is
touched by any drift commit. The NOTICE.md delta at the drift tip describes
components bundled in the upstream repository only.

## 4. Dependency-policy impact

No change to `docs/port/dependency-policy.json` today.
`tools/port/check_dependencies.py` enforces the policy over the three Rust
manifests listed in the policy file; nothing in the drift adds, removes, or
relicenses any crate in those graphs. Upstream's own Rust decisions (default
Rust 1.96.0, vendored `log`/`paste` under `libs/3rdparty/cargo-vendor/`) are
upstream-repo-internal and would not enter this workspace even at a re-pin,
because the oracle is consumed as a behavioral reference, not as a dependency.
`forbid_git_dependencies` remains satisfied; the allowed-license list needs no
extension.

## 5. Items deferred to re-pin (feeds U06)

A future re-pin at or beyond `be0029bb` must, in the same governed change:

1. Update the bundled-component table in `docs/port/upstream-baseline.md`:
   ETL 20.48.1, CMSIS path `libs/3rdparty/cmsis/LICENSE`, and the two new ST
   STM32F4 (2.6.11) / STM32G4 (1.2.6) Apache-2.0 header rows. NOTICE
   obligations for shipped Rust artifacts still would not change unless
   upstream source is actually copied into the workspace.
2. Re-audit any source cited or copied from upstream against the automated
   Eclipse copyright headers (`c21dcc3a`): header text on referenced upstream
   files changed en masse, so header citations in port documentation may
   need refresh.
3. Account for oracle-build changes: minimum C++17 (`e45eee34`), the
   Bazel-primary build layout, and the middleware build-time code generation
   (`c4401a13`) affect how `tools/port/openbsw_oracle.ps1` would need to
   build a newer oracle.
4. ETL 20.46.2 -> 20.48.1 interface changes (plus the estd-to-ETL
   migrations in DoIP/ETH) alter upstream internals and possibly observable
   behavior surfaces; oracle fixtures and parity evidence would need
   regeneration under the re-pin gate, not before.
5. If the Rust STM32 BSPs ever derive register definitions from upstream's
   new bundled ST headers, the ST Apache-2.0 copyright must be carried into
   this port's NOTICE documentation. Today the BSPs do not.

## 6. State attestation

- Oracle checkout `target/oracle/openbsw`: absent before and after this
  review; left absent. Parity ledger (`parity-manifest.json`), `status.md`,
  and all release/evidence documents untouched.
- `docs/port/sbom.cdx.json` unchanged;
  `python tools/port/generate_sbom.py --output docs/port/sbom.cdx.json
  --check` passes after this review.
- `docs/port/dependency-policy.json` unchanged.
- The only file created by this review is this memo.
