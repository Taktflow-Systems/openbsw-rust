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

## Optional LAUNCHXL2-570LC43 target

The connected silicon was identified without opening a CPU session: one
XDS110-class path exposed one six-bit ICEPick-C chain, and a TAP-only scan
returned public JTAG ID `0x1b95a02f`. SPNS195C table 6-52 identifies that code
as TMS570LC4357 revision B. Raw probe selectors and topology logs remain under
ignored `target/private-evidence/` and must never enter commands or reports.

Query-only enumeration identifies an embedded
XDS110 in Standard configuration, with four correlated USB functions including
debug and serial interfaces. TI's installed desktop detector finds the probe
but returns no board name because its board-ID database has no LC43/RM57
LaunchPad entry. Its private board-prefix comparison is therefore
inconclusive; neither the prefix nor the serial may be published. The current
cloud UniFlash path requires a myTI login before detection, so no credentials,
permissions, or device session were provided. The operator subsequently
identified the connected carrier from its procurement record as TI product
`LAUNCHXL2-570LC43`; no invoice, shipment, tracking, or order metadata is
retained. The separate host Ethernet adapter has an active 100 Mbit/s link
partner emitting small Layer-2 broadcasts, but no IPv4 or neighbor identity;
this does not independently establish PHY ownership or DoIP.

Feature `launchxl2-570lc43` selects only the public SPRR397 A1 wiring: a
16 MHz MCU crystal, four-bit MII, DP83630 at MDIO address 1, 25 MHz ECLK1T
reference-clock requirement, active-low PHY reset on GIOA[4], active-low
power-down/interrupt on GIOA[3] with an installed pull-down, and the direct
RJ-45 magnetics. No writable flash region is selected.

The operator confirmed that the connected target is a bare board. A separately
reviewed 12-byte register-only image was then loaded into SRAM and passed two
clean-start marker runs after a controlled reset; no flash was erased or
programmed. This proves only debugger RAM load and A32 execution. Flash remains
prohibited until the current application, protection state, exact geometry,
rebuilt map, complete image range, recoverable image/authorization, and
intended writable region are proven. GPIO, PWM, CAN, watchdog, and PHY-control
work still requires signal-specific safety review.

A second SRAM-only role has also passed twice from controlled reset. It applies
the bounded LC4357 PLL advisory sequence, measures PLL1 with DCC before source
selection, and establishes 300-MHz GCLK, 150-MHz HCLK, and 75-MHz VCLK/VCLK2/
VCLK3. It does not configure ECLK, pinmux, PHY control, EMAC, or any external
board signal.

A third 224-byte role is loaded into a disjoint SRAM window only after that
clock role passes. Two current-artifact clean starts proved privileged CPSR
IRQ/FIQ mask-and-restore, RTI counter-0 internal timebase, CPUC0=74, and at
least 100,000 one-microsecond ticks. The CPU is left halted after observation.
Counter 1, DWWD, pinmux, and external board signals remain untouched.

A fourth 1,152-byte SRAM-only role is staged below the RTI window after the
same clock qualification. It validates the resident fixed IRQ/FIQ flash words
as standard read-only VIM trampolines, initializes VIM RAM ECC, installs only
RTI compare-0 channel 2 in VIM RAM, and uses separate bounded IRQ/FIQ stacks.
Two controlled-reset runs delivered one IRQ and one FIQ with hardware index 3.
Revision-B DEVICE#56 recovery clears only Group-2 channel 2 in live and shadow
ESM status and verifies all unrelated ESM status words are unchanged. No
flash, pinmux, GPIO, CAN, watchdog, PHY, EMAC, or external signal is touched.

A fifth 1,836-byte SRAM-only role is staged below the VIM window. It installs
private Undefined, Abort, Supervisor, and System stacks and arms a 160-byte
integrity-protected record. For each of UDF, SVC, an unimplemented instruction
fetch, and an unimplemented data read, the debugger catches the fixed resident
vector after banked architectural entry and redirects only PC to the shared
RAM handler. Two controlled-reset runs per class preserved and classified the
same committed record at reset vector zero (8/8). No flash vector was changed;
no flash, pinmux, GPIO, CAN, watchdog, PHY, EMAC, or external signal was
touched. Raw selectors and records remain private.
