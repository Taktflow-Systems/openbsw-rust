# OpenBSW Rust parity status

> Generated from `parity-manifest.json` by `tools/port/generate_status.py`; do not edit by hand.

Pinned upstream: `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`

Mandatory rows closed: **33/37**. A row closes only as `done` or `native replacement`; `partial` never counts.

| Scope | Done | Native replacement | Partial | Missing | Excluded |
|---|---:|---:|---:|---:|---:|
| mandatory | 22 | 11 | 0 | 3 | 1 |
| optional | 0 | 0 | 0 | 0 | 1 |
| project extension | 3 | 0 | 0 | 0 | 0 |

| Row | Status | Strategy | Rust crates | Owner packages |
|---|---|---|---|---|
| `bsw.async` | native replacement | native replacement | `bsw-async` | C09-C11 |
| `bsw.asyncConsole` | native replacement | port behavior | `bsw-console` | C16-C18 |
| `bsw.asyncFreeRtos` | native replacement | native replacement | `bsw-async` | C09-C11 |
| `bsw.asyncImpl` | native replacement | native replacement | `bsw-async` | C09-C11 |
| `bsw.asyncThreadX` | excluded | FreeRTOS/POSIX Rust executors replace ThreadX | — | C09-C11 |
| `bsw.bsp` | done | port contracts and implement per board | `bsw-bsp-stm32` | D03, G01-G20 |
| `bsw.common` | native replacement | native replacement | `bsw-common` | C01 |
| `bsw.cpp2can` | done | port behavior | `bsw-can` | D12-D16 |
| `bsw.cpp2ethernet` | done | port behavior | `bsw-ethernet` | D17-D21 |
| `bsw.docan` | done | port behavior | `bsw-docan` | E09-E13 |
| `bsw.doip` | done | port behavior | `bsw-doip` | E26-E31 |
| `bsw.estd` | native replacement | native Rust fixed-capacity types | `bsw-estd` | C02-C04 |
| `bsw.io` | done | port behavior | `bsw-io` | D04-D05 |
| `bsw.lifecycle` | done | port behavior | `bsw-lifecycle` | D01-D02 |
| `bsw.logger` | native replacement | native replacement | `bsw-logger` | C13-C14 |
| `bsw.loggerIntegration` | native replacement | native replacement | `bsw-logger` | C13-C14 |
| `bsw.lwipSocket` | native replacement | port boundary | `bsw-ethernet` | D20 |
| `bsw.middleware` | done | port behavior | `bsw-middleware` | E05-E08 |
| `bsw.platform` | done | native platform traits | `bsw-platform`, `bsw-bsp-stm32` | D03, F01, G01 |
| `bsw.runtime` | done | port behavior | `bsw-runtime` | C12 |
| `bsw.stdioConsoleInput` | native replacement | native replacement | `bsw-console` | C15-C17 |
| `bsw.storage` | done | port behavior | `bsw-storage`, `bsw-bsp-stm32` | D06-D11, G11 |
| `bsw.timer` | done | native replacement | `bsw-time`, `bsw-bsp-stm32` | C07-C08, G05 |
| `bsw.transport` | done | port behavior | `bsw-transport` | E01-E02 |
| `bsw.transportRouterSimple` | done | port behavior | `bsw-transport` | E03-E04 |
| `bsw.uds` | done | port behavior | `bsw-uds` | E14-E25 |
| `bsw.util` | native replacement | native Rust utilities | `bsw-util`, `bsw-common`, `bsw-estd` | C05-C06 |
| `bsp.bspCharInputOutput` | done | native console traits | `bsw-console`, `bsw-bsp-stm32` | C18, G06 |
| `bsp.bspDynamicClient` | missing | port behavior | — | G01-G15 |
| `bsp.bspInputManager` | done | native platform traits | `bsw-bsp-stm32` | G07, G09 |
| `bsp.bspInterrupts` | done | native Cortex-M implementation | `bsw-bsp-stm32` | G02-G04 |
| `bsp.bspOutputManager` | done | native platform traits | `bsw-bsp-stm32` | G07 |
| `bsp.bspOutputPwm` | done | native platform traits | `bsw-bsp-stm32` | G08 |
| `safety.safeMonitor` | missing | port behavior | — | H01-H09 |
| `safety.safeUtils` | missing | port behavior | — | H01-H09 |
| `platform.posix` | done | native POSIX adapters | `bsw-platform-posix` | F01 |
| `app.referenceApp` | done | compose equivalent Rust application | `openbsw-reference-app` | F02-F10 |
| `platform.s32k1xx` | excluded | optional expansion after mandatory release | — | S01-S18 |
| `extension.com` | done | retain and harden | `bsw-com` | E32-E33 |
| `extension.e2e` | done | label and harden | `bsw-util`, `bsw-com` | E34 |
| `extension.can-fd` | done | retain and harden | `bsw-can` | D14 |
