# Final mandatory port release notes - 2026-07-19

This release completes packages G12-G20, H01-H09, and I01-I09 and advances the
mandatory Eclipse openBSW Rust port to 1,092/1,092 package-hours and 37/37
mandatory parity rows. The parity baseline remains
`ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`.

Highlights:

- linker-reserved, power-loss-safe storage is complete on F413 and G474, with
  host cut-point enumeration and physical transition-class recovery campaigns;
- F413/G474 production applications share heap-free application state and the
  same DoCAN/UDS/lifecycle/storage composition;
- deterministic privacy-safe HIL covers clean boot, clock, console, I/O, CAN,
  UDS single/multiframe, reset, storage, overflow, bus-off, malformed traffic,
  resource exhaustion, repeated reset, and bounded soak;
- resource limits now enforce whole-image flash/BSS, static buffers, queue
  capacities, call-path stack, ISR nesting reserve, timing, and soak duration;
- the `bsw-safety` crate adds tested monitor/supervisor/watchdog/retained-event,
  incremental ROM CRC, MPU/ECC capability, and composed lifecycle mechanisms;
- the release includes architecture, developer, board, test, SBOM, license,
  NOTICE, reproducible-build, drift, and final-acceptance documentation.

Release artifacts are independently reproducible for POSIX, STM32F413, and
STM32G474. Commands, publishable sizes, comparison outcomes, regression counts,
and HIL summaries are indexed by the final matrix. Exact source/artifact
digests, maps, and binaries remain in its referenced ignored private evidence.

Known limitations are explicit: embedded DoIP is excluded because these boards
have no supported Ethernet PHY path; PWM was not externally oscilloscope-
measured; physical storage interruption is deterministic controller-operation
injection and reset/remount, not an uncontrolled supply cut; safety mechanisms
are not a certification claim; and the archived transitive `bare-metal` crate
has no safe upgrade in the current Cortex-M dependency line.

Current upstream is 166 commits ahead of the pinned baseline. I08 records the
delta and follow-up packages in `upstream-drift-2026-07-18.md`; this release does
not silently rebase its oracle.
