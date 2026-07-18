# STM32 F413/G474 convergence (G01-G11)

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

## Target evidence

The target run used probe-class-only metadata; no probe or device serial number
is tracked. Results were:

- STM32G474 storage: 8 checks passed; wear counters showed an area relocation.
- STM32G474 application: PWM setup passed, ADC conversion returned 1,536, and
  watchdog/reset/UART/GPIO startup passed.
- STM32F413 application: PWM setup passed, ADC conversion returned 2,048, and
  watchdog/reset/UART/GPIO startup passed.

The G474 storage test is deliberately destructive only within linker-reserved
`0x0807E000..0x0807FFFF`. The retired fixed-layout `nvm.rs` implementation is
not compiled; production G474 storage uses `G4NvmBackend` plus
`bsw_storage::journal::JournalStore`.

## Limits

PWM register programming and application startup were verified on both boards,
and frequency/duty arithmetic is boundary-tested on the host. An external
oscilloscope measurement of the waveform was not captured. G12 (F413 storage)
and later whole-program stack/ISR budget packages are outside this checkpoint.
