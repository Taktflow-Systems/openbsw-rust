# F413 and G474 board setup

Both reference boards use 115200 8N1 UART and 500 kbit/s classic CAN. Connect
the CAN nodes to a correctly terminated bench. `run_physical_matrix.ps1`
discovers probes by class and MCU IDCODE, correlates each probe to its virtual
COM interface through the PnP container, and proves which of duplicate targets
is connected to a project-configured hardware SocketCAN fixture. It proceeds
only when exactly one F413/G474/fixture topology responds. The selected values
remain process-local; reports contain only generic fixture and probe classes.
Non-selected peers are placed in the non-destructive blink role.

`hil/deterministic_fixture.py` verifies the release download, resets the CAN
adapter state, captures monotonic timestamps, and requires two agreeing runs.
F413 storage is restricted by linker symbols to sectors 14-15
(`[0x08140000, 0x08180000)`). G474 storage is the linker-reserved top 8 KiB
(`[0x0807e000, 0x08080000)`). The runner rechecks exact-artifact linker symbols
immediately before flashing; only dedicated storage/safety roles may erase
within those half-open ranges. Mass erase is forbidden.

Run the complete matrix against an immutable private artifact set:

```powershell
powershell -File tools/port/run_physical_matrix.ps1 `
  -ArtifactRoot target/private-evidence/<release>/selected `
  -OutputRoot target/private-evidence/<release-run>
```

F413 has no claimed ECC diagnostic. G474 exposes flash ECC status observation
only. Neither board has an on-board Ethernet path in this reference setup;
DoIP parity is authoritative on POSIX.
