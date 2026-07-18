# Pinned upstream test inventory

OpenBSW commit: `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`

Counts include C/C++ source and header files below each candidate root. A zero means the behavior needs implementation-derived or integration evidence; it does not mean the module is out of scope.

| Parity row | Status | Upstream path | Candidate files | Candidate roots |
|---|---|---|---:|---|
| `bsw.async` | missing | `libs/bsw/async` | 4 | `libs/bsw/async/test` |
| `bsw.asyncConsole` | missing | `libs/bsw/asyncConsole` | 4 | `libs/bsw/asyncConsole/test` |
| `bsw.asyncFreeRtos` | missing | `libs/bsw/asyncFreeRtos` | 17 | `libs/bsw/asyncFreeRtos/test` |
| `bsw.asyncImpl` | missing | `libs/bsw/asyncImpl` | 7 | `libs/bsw/asyncImpl/test` |
| `bsw.asyncThreadX` | excluded | `libs/bsw/asyncThreadX` | 0 | — |
| `bsw.bsp` | partial | `libs/bsw/bsp` | 3 | `libs/bsw/bsp/test` |
| `bsw.common` | missing | `libs/bsw/common` | 0 | — |
| `bsw.cpp2can` | partial | `libs/bsw/cpp2can` | 5 | `libs/bsw/cpp2can/test` |
| `bsw.cpp2ethernet` | partial | `libs/bsw/cpp2ethernet` | 11 | `libs/bsw/cpp2ethernet/test` |
| `bsw.docan` | partial | `libs/bsw/docan` | 22 | `libs/bsw/docan/test` |
| `bsw.doip` | partial | `libs/bsw/doip` | 31 | `libs/bsw/doip/test` |
| `bsw.estd` | partial | `libs/bsw/estd` | 0 | — |
| `bsw.io` | partial | `libs/bsw/io` | 6 | `libs/bsw/io/test` |
| `bsw.lifecycle` | partial | `libs/bsw/lifecycle` | 7 | `libs/bsw/lifecycle/test` |
| `bsw.logger` | missing | `libs/bsw/logger` | 12 | `libs/bsw/logger/test` |
| `bsw.loggerIntegration` | missing | `libs/bsw/loggerIntegration` | 0 | — |
| `bsw.lwipSocket` | missing | `libs/bsw/lwipSocket` | 1 | `libs/bsw/lwipSocket/test` |
| `bsw.middleware` | missing | `libs/bsw/middleware` | 2 | `libs/bsw/middleware/test` |
| `bsw.platform` | partial | `libs/bsw/platform` | 4 | `libs/bsw/platform/test` |
| `bsw.runtime` | partial | `libs/bsw/runtime` | 12 | `libs/bsw/runtime/test` |
| `bsw.stdioConsoleInput` | missing | `libs/bsw/stdioConsoleInput` | 0 | — |
| `bsw.storage` | partial | `libs/bsw/storage` | 1 | `libs/bsw/storage/test` |
| `bsw.timer` | partial | `libs/bsw/timer` | 1 | `libs/bsw/timer/test` |
| `bsw.transport` | partial | `libs/bsw/transport` | 7 | `libs/bsw/transport/test` |
| `bsw.transportRouterSimple` | missing | `libs/bsw/transportRouterSimple` | 0 | — |
| `bsw.uds` | partial | `libs/bsw/uds` | 48 | `libs/bsw/uds/test` |
| `bsw.util` | partial | `libs/bsw/util` | 46 | `libs/bsw/util/test` |
| `bsp.bspCharInputOutput` | missing | `libs/bsp/bspCharInputOutput` | 0 | — |
| `bsp.bspDynamicClient` | missing | `libs/bsp/bspDynamicClient` | 0 | — |
| `bsp.bspInputManager` | missing | `libs/bsp/bspInputManager` | 0 | — |
| `bsp.bspInterrupts` | partial | `libs/bsp/bspInterrupts` | 0 | — |
| `bsp.bspOutputManager` | missing | `libs/bsp/bspOutputManager` | 0 | — |
| `bsp.bspOutputPwm` | missing | `libs/bsp/bspOutputPwm` | 0 | — |
| `safety.safeMonitor` | missing | `libs/safety/safeMonitor` | 0 | — |
| `safety.safeUtils` | missing | `libs/safety/safeUtils` | 0 | — |
| `platform.posix` | missing | `platforms/posix` | 39 | `platforms/posix` |
| `app.referenceApp` | missing | `executables/referenceApp` | 154 | `executables/referenceApp` |
| `platform.s32k1xx` | excluded | `platforms/s32k1xx` | 126 | `platforms/s32k1xx` |

Total candidate files (roots may intentionally overlap): **570**.
