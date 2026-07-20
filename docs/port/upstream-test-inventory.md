# Pinned upstream test inventory

OpenBSW commit: `be0029bbb79fe901048a24c2665f2ba854328734`

Counts include C/C++ source and header files below each candidate root. A zero means the behavior needs implementation-derived or integration evidence; it does not mean the module is out of scope.

| Parity row | Status | Upstream path | Candidate files | Candidate roots |
|---|---|---|---:|---|
| `bsw.async` | native replacement | `libs/bsw/async` | 4 | `libs/bsw/async/test` |
| `bsw.asyncConsole` | native replacement | `libs/bsw/asyncConsole` | 4 | `libs/bsw/asyncConsole/test` |
| `bsw.asyncFreeRtos` | native replacement | `libs/bsw/asyncFreeRtos` | 17 | `libs/bsw/asyncFreeRtos/test` |
| `bsw.asyncImpl` | native replacement | `libs/bsw/asyncImpl` | 7 | `libs/bsw/asyncImpl/test` |
| `bsw.asyncThreadX` | native replacement | `libs/bsw/asyncThreadX` | 0 | — |
| `bsw.bsp` | done | `libs/bsw/bsp` | 3 | `libs/bsw/bsp/test` |
| `bsw.common` | native replacement | `libs/bsw/common` | 0 | — |
| `bsw.cpp2can` | done | `libs/bsw/cpp2can` | 5 | `libs/bsw/cpp2can/test` |
| `bsw.cpp2ethernet` | done | `libs/bsw/cpp2ethernet` | 11 | `libs/bsw/cpp2ethernet/test` |
| `bsw.docan` | done | `libs/bsw/docan` | 22 | `libs/bsw/docan/test` |
| `bsw.doip` | done | `libs/bsw/doip` | 31 | `libs/bsw/doip/test` |
| `bsw.estd` | native replacement | `libs/3rdparty/etl` | 0 | — |
| `bsw.io` | done | `libs/bsw/io` | 6 | `libs/bsw/io/test` |
| `bsw.lifecycle` | done | `libs/bsw/lifecycle` | 7 | `libs/bsw/lifecycle/test` |
| `bsw.logger` | native replacement | `libs/bsw/logger` | 12 | `libs/bsw/logger/test` |
| `bsw.loggerIntegration` | native replacement | `libs/bsw/loggerIntegration` | 0 | — |
| `bsw.lwipSocket` | native replacement | `libs/bsw/lwipSocket` | 1 | `libs/bsw/lwipSocket/test` |
| `bsw.middleware` | done | `libs/bsw/middleware` | 32 | `libs/bsw/middleware/test` |
| `bsw.platform` | done | `libs/bsw/platform` | 4 | `libs/bsw/platform/test` |
| `bsw.runtime` | done | `libs/bsw/runtime` | 12 | `libs/bsw/runtime/test` |
| `bsw.stdioConsoleInput` | native replacement | `libs/bsw/stdioConsoleInput` | 0 | — |
| `bsw.storage` | done | `libs/bsw/storage` | 1 | `libs/bsw/storage/test` |
| `bsw.timer` | done | `libs/bsw/timer` | 1 | `libs/bsw/timer/test` |
| `bsw.transport` | done | `libs/bsw/transport` | 5 | `libs/bsw/transport/test` |
| `bsw.transportRouterSimple` | done | `libs/bsw/transportRouterSimple` | 0 | — |
| `bsw.uds` | done | `libs/bsw/uds` | 49 | `libs/bsw/uds/test` |
| `bsw.util` | native replacement | `libs/bsw/util` | 36 | `libs/bsw/util/test` |
| `bsp.bspCharInputOutput` | done | `libs/bsp/bspCharInputOutput` | 0 | — |
| `bsp.bspDynamicClient` | native replacement | `libs/bsp/bspDynamicClient` | 0 | — |
| `bsp.bspInputManager` | done | `libs/bsp/bspInputManager` | 0 | — |
| `bsp.bspInterrupts` | done | `libs/bsp/bspInterrupts` | 0 | — |
| `bsp.bspOutputManager` | done | `libs/bsp/bspOutputManager` | 0 | — |
| `bsp.bspOutputPwm` | done | `libs/bsp/bspOutputPwm` | 0 | — |
| `safety.safeMonitor` | done | `libs/safety/safeMonitor` | 0 | — |
| `safety.safeUtils` | native replacement | `libs/safety/safeUtils` | 0 | — |
| `platform.posix` | done | `platforms/posix` | 39 | `platforms/posix` |
| `app.referenceApp` | done | `executables/referenceApp` | 169 | `executables/referenceApp` |
| `platform.s32k1xx` | excluded | `platforms/s32k1xx` | 118 | `platforms/s32k1xx` |
| `bsw.routing` | excluded | `libs/bsw/routing` | 24 | `libs/bsw/routing/test` |
| `bsw.blob` | excluded | `libs/bsw/blob` | 3 | `libs/bsw/blob/test` |
| `bsw.time` | done | `libs/bsw/time` | 1 | `libs/bsw/time/test` |

Total candidate files (roots may intentionally overlap): **624**.
