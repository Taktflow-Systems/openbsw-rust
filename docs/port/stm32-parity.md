# STM32 F413/G474 convergence (G01-G20)

The implementation is grounded in the pinned OpenBSW baseline and the G-phase
survey snapshot. It ports the board-level behavior and contracts; it does not
claim source-level identity with the upstream BSP.

| Package | Result | Primary evidence |
|---|---|---|
| G01 | done | `board.rs` is the single pin/clock table and issues unique peripheral tokens; ARM builds require exactly one MCU feature. |
| G02 | done | `mmio.rs` is the volatile register boundary; all 15 examples are role-only entrypoints checked by `check_stm32_examples.py`. |
| G03 | done | both bxCAN and FDCAN feed `InterruptQueue` and expose the shared `bsw_can::FrameSource` receive contract. |
| G04 | done | mock-register/W1C, overflow, error-passive, bus-off, and exact injected-deadline recovery tests; both target drivers use `CanHealth`. |
| G05 | done | one DWT `Clock`/`SystemTimer` implementation with remainder and 32-bit wrap tests; board startup asserts the measured clock. |
| G06 | done | both UARTs implement bounded nonblocking `CharInput`/`CharOutput`; applications use `UartWriter<_, 256>`. |
| G07 | done | common GPIO input/output/debounce managers and role-shared LED application; output readback uses ODR. |
| G08 | done | TIM3_CH1/PA6 PWM on both boards, with frequency/duty boundary tests and successful on-target setup. |
| G09 | done | ADC1/PA0 on both boards, with calibration/range/error tests and on-target conversions. |
| G10 | done | watchdog/reset/deferred-reset/critical-section/fault-retention behavior is shared; reset cause is logged after reboot and NOINIT is linker-owned. |
| G11 | done | G474 reserves the top 8 KiB, uses the generic journal backend, masks interrupts around flash mutation, bounds busy waits, checks DBANK, and passed the destructive target conformance runner. |
| G12 | done | F413 reserves sectors 14-15, validates linker symbols, bounds/masks flash operations, and passed wear plus old/new recovery conformance. |
| G13 | done | final flash/BSS/buffer/task/ISR/call-path budgets are checked by `measure_resources.ps1 -Check`. |
| G14-G15 | done | both release applications use `bsw-reference-core` and shared protocol/application composition; board roles contain no protocol implementation. |
| G16-G18 | done | privacy-safe topology discovery, strict schemas, CAN/reset/NOINIT isolation, monotonic timestamps, exact artifacts, and two agreeing ten-case runs per board. |
| G19 | done | a reset/reboot occurs after each relocation-header/payload/commit, new-record-header/payload/commit, and source-area-erase cut; recovery precedes nominal erase and every mount yields old or new data. |
| G20 | done | each MCU proves real FIFO overflow plus valid traffic, error-passive/bus-off plus timed recovery and repeated valid traffic, occupied-session exhaustion/recovery, ten resets, and a 6,000-request ten-minute soak. |

## Target evidence

The target run used probe-class-only metadata; no probe or device serial number
is tracked. Results were:

- Each board passed two agreeing application smokes, all seven reset-safe
  journal transition cuts, and the complete H01-H08 physical fault matrix.
- Both boards reported nonzero hardware FIFO overflow and then answered valid
  diagnostic traffic.
- Both real CAN controllers entered error-passive and BusOff under a
  TX-isolated no-ACK injection, recovered within the 1.1-second threshold, and
  emitted repeated valid traffic after recovery.
- Both boards accepted at least 64 fresh requests after a 256-frame burst,
  recovered after an actually occupied static session, produced ten
  consecutive ready boots, and answered all 6,000 requests in the ten-minute
  10 Hz soak.

The storage tests are destructive only inside the exact linker-reserved ranges:
F413 `[0x08140000, 0x08180000)` and G474 `[0x0807E000, 0x08080000)`. The runner
validates those symbols immediately before each flash. The retired fixed-layout
`nvm.rs` implementation is not compiled; production storage uses each board
backend plus `bsw_storage::journal::JournalStore`.

## Limits

PWM register programming and application startup were verified on both boards,
and frequency/duty arithmetic is boundary-tested on the host. An external
oscilloscope measurement of the waveform was not captured. DoIP remains a
POSIX parity claim because neither reference board exposes the required
Ethernet PHY. Physical power interruption is controller-operation injection at
the journal backend and reset/remount boundary, not an uncontrolled bench power
pull. Safety and ECC limits are in the non-certification mechanism report.
