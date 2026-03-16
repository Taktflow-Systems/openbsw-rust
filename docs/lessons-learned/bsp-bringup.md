# Lessons Learned — Rust BSP Bringup (openbsw-rust)

## 2026-03-15 — Three hardware-config bugs on STM32 NUCLEO boards

**Context**: First hardware test of pure Rust BSP (`bsw-bsp-stm32`) on NUCLEO-G474RE and NUCLEO-F413ZH. Code compiled clean for both features. DWT timer, state machines, CAN drivers all type-checked. LED blink loop confirmed running via SWD register read.

**Bug 1 — G4 PLL divider encoding**: STM32G4 RCC_PLLCFGR PLLR bits [26:25] use encoding `00=/2, 01=/4, 10=/6, 11=/8` — NOT `value-1`. Code had `(2-1) << 25 = 1 << 25`, which set PLLR to /4 instead of /2. SYSCLK was 85 MHz instead of 170 MHz. UART output was garbled at exactly half the expected baud rate.
**Fix**: Changed to `0b00 << 25` for /2. Confirmed via SWD register readback.
**Principle**: STM32 PLL divider registers use inconsistent encodings — some are `value-1` (PLLM, PLLN), some are enumerated (PLLP, PLLR). Always verify the encoding table in the reference manual for each field individually. Don't assume a pattern.

**Bug 2 — G4 UART missing GPIO alternate function config**: UART init enabled the peripheral clock and configured USART2 registers (BRR, CR1) but did not configure PA2/PA3 as AF7. Pins were in default analog mode → no signal on the wire. USART2 registers read back correctly via SWD (SR=0xC0, BRR=0x5C4, CR1 correct), but no output because GPIO was wrong.
**Fix**: Added GPIOA clock enable + MODER/OSPEEDR/AFRL configuration for PA2/PA3 AF7 in `init()`.
**Principle**: When UART registers look correct but no output appears, check GPIO configuration first. USART will happily "transmit" into an unconfigured pin. Always configure GPIO AF before enabling the peripheral.

**Bug 3 — F4 UART wrong USART peripheral**: NUCLEO-F413ZH (144-pin) routes ST-LINK VCP to **USART3 on PD8/PD9**, not USART2 on PA2/PA3. The 64-pin NUCLEO boards (like G474RE) use USART2/PA2/PA3 for VCP. Code was using USART2 — registers configured correctly but output went to Arduino connector pins, not to VCP.
**Fix**: Changed from USART2/PA2/PA3 to USART3/PD8/PD9 (AF7). PD8/PD9 use AFRH (not AFRL).
**Principle**: NUCLEO board VCP routing depends on package size. 64-pin → USART2/PA2/PA3. 144-pin → USART3/PD8/PD9. Always check the board schematic/user manual (UM1974/UM2010) for VCP wiring, don't assume USART2.

**Debugging approach that worked**: (1) Read core registers via SWD to confirm MCU is running and where PC is. (2) Read peripheral registers (RCC_PLLCFGR, USART_SR/BRR/CR1, GPIO_MODER/AFRL) to verify configuration. (3) Calculate actual frequencies from register values. The SWD register dump immediately identified PLLR=01 → /4 and showed UART was configured but GPIO wasn't.

## 2026-03-16 — DWT timer integer division truncation kills scheduler + watchdog

**Context**: Full Rust BSW app on G474RE with DWT-based system timer, cooperative scheduler (1ms/10ms/100ms tasks), and IWDG watchdog kicked from 1ms task. Watchdog kept resetting the board every 1 second despite scheduler appearing to work.

**Mistake**: `DwtTimer::update()` computed `ticks_us += elapsed_cycles / freq_mhz`. At 170 MHz, `freq_mhz = 170`. In a tight main loop, the DWT cycle counter advances only ~68 cycles between calls. `68 / 170 = 0` (integer division truncation). The microsecond accumulator stopped advancing in tight loops, so no scheduler tasks ever became "due", and the watchdog was never kicked.

**Why it appeared to work in the blink example**: The blink example used `system_time_us_64()` with a simple LED toggle — the loop body was heavier (volatile GPIO write + branch), causing DWT to advance ~200+ cycles between calls, just enough for `200/170 = 1` to not truncate to zero.

**Fix**: Track remainder cycles: `total_cycles = elapsed + remainder; ticks_us += total/freq_mhz; remainder = total % freq_mhz`. This ensures sub-microsecond cycle fractions accumulate correctly across calls.

**Principle**: Never divide a monotonic counter by a fixed rate without handling the remainder. Integer division truncation in cycle-to-time conversion is invisible until the loop body is faster than one time quantum (1us at MHz-scale). Always track the remainder, or accumulate raw cycles and convert lazily.

**Also fixed**: IWDG init sequence — must write `KEY_ENABLE` (0xCCCC) before `KEY_UNLOCK` (0x5555) to start the LSI oscillator; otherwise the PVU/RVU wait for prescaler/reload register update hangs because LSI isn't clocked.

## 2026-03-16 — STM32G4 FDCAN uses different register map from Bosch M_CAN

**Context**: FDCAN CAN loopback test on NUCLEO-G474RE — init succeeded, TX frame sent, but RX FIFO 0 stayed empty for 500ms timeout. The driver was a direct Rust translation of the C++ `FdCanDevice.cpp` which used generic M_CAN register offsets.

**Mistake**: Assumed the STM32G4 FDCAN uses the same register layout as the generic Bosch M_CAN IP core. It doesn't. ST implemented a **simplified** FDCAN for the G4 family with:
1. **Shifted register offsets**: RXF0S at 0x090 (not 0x0A4), TXFQS at 0x0C4 (not 0x0C8), TXBAR at 0x0CC (not 0x0D0), TXBC at 0x0C0 (not 0x0C4).
2. **Non-existent registers**: SIDFC (0x084), XIDFC (0x088), RXF0C (0x0A0), RXF1C (0x0B0), TXEFC (0x0F0) — these are XIDAM, HPMS, reserved, reserved, TXEFA on G4. Writing to them corrupted other registers (XIDAM) or was silently ignored.
3. **Hardware-fixed MRAM layout**: no configurable start-address registers. Each FDCAN1 section has a fixed offset from SRAMCAN_BASE.
4. **18-word (72-byte) element stride**: RX/TX elements are 18 words each (FD-size), not 4 words (classic CAN size). TX data written at wrong MRAM offset, RX data read from wrong offset.
5. **Narrower bit fields**: RXF0S fill-level is 4 bits [3:0] (not 7), get-index is 2 bits [9:8] (not 6 bits).

**Symptoms**: TXBAR write at 0x0D0 → hit wrong register, TX never requested. RXF0S read at 0x0A4 → reserved, always 0 → "no frames received". SIDFC write at 0x084 → corrupted XIDAM. Register dump showed TXBC=0x00000003 (reading TXFQS at the wrong offset).

**Fix**: Updated all register offsets to match STM32G4 CMSIS `FDCAN_GlobalTypeDef`. Removed writes to non-existent registers. Changed MRAM element stride from 16 → 72 bytes. Added TXBC configuration (reset value is 0 = no TX buffers).

**Porting rule**: Every time you port FDCAN to a new STM32 family, verify: (1) register offsets against CMSIS `FDCAN_GlobalTypeDef` struct, (2) which config registers exist (SIDFC/XIDFC/RXF0C may not), (3) MRAM element stride, (4) bit field widths in status registers. Start from the target chip's CMSIS/PAC headers, never from a "generic" M_CAN spec.

**Principle**: Never assume peripheral register maps are portable across MCU families, even within the same vendor. The C++ code had the same bug — it compiled fine because register offsets are just integer constants. Always validate with a diagnostic register dump on real hardware.
