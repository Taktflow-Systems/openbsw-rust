# Current-upstream drift assessment - 2026-07-19

The release parity baseline remains Eclipse openBSW commit
`ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`. The read-only oracle checkout is
still at that revision; its two pre-existing local lwIP documentation/header
changes were not modified.

The current public `main` tip observed during I08 was
`be0029bbb79fe901048a24c2665f2ba854328734`. A clean ignored bare fetch reports
that tip as 166 commits ahead and zero behind the pinned baseline, spanning
2026-03-05 through 2026-06-02. The exhaustive local comparison contains 3,556
changed paths, including 1,854 paths selected by the mandatory-surface routing
rule; the pinned read-only oracle checkout was not moved.

| Delta area | Commit-subject signal | Release decision |
|---|---:|---|
| build/toolchain | 25 | follow up; no effect on the pinned behavioral oracle |
| UDS/diagnostics | 5 | review service and routing changes against shared UDS |
| DoCAN/ISO-TP | 1 | replay any added transport vectors |
| DoIP | 18 | review entity and diagnostic-routing deltas |
| CAN/STM32 | 2 direct CAN subjects, including new STM32 adapters | compare upstream adapter semantics with the native F4/G4 drivers |
| reference application | 3 | compare composition and observable workflows |
| storage/safety | 3 storage signals; no safety-subject signal | no evidence requiring a baseline move; inspect storage deltas separately |
| async/executors | 4 | compare scheduling contracts without adopting platform-specific ThreadX code |

The following non-release-blocking follow-up packages capture the drift:

- U01: refresh the detailed module/file survey against the exhaustive
  166-commit, 3,556-path delta retained in the ignored drift checkout.
- U02: import and differential-test new UDS, DoCAN, and DoIP vectors.
- U03: compare new upstream STM32 CAN/transceiver behavior with the Rust
  bxCAN/FDCAN health, overflow, and bus-off contracts.
- U04: review middleware and reference-application composition changes.
- U05: review build, ETL, license, and NOTICE changes for dependency-policy
  impact.
- U06: decide whether a future release should deliberately re-pin, only after
  U01-U05 close and the full regression matrix is rerun.

I08 therefore records material upstream drift while intentionally leaving the
release baseline and the completed mandatory parity ledger unchanged.
