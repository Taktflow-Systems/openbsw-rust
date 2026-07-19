# Production reference architecture

The production path has one heap-free application core and one protocol stack.
`bsw-reference-core` owns application state and diagnostic behavior;
`bsw-uds`, `bsw-docan`, and `bsw-doip` provide the shared transport-independent
UDS implementation. POSIX and STM32 compose those crates through platform
adapters. Board examples select only a role and contain no protocol logic.

The dependency direction is:

```text
reference application/core
  -> lifecycle, I/O, storage, console, UDS
  -> DoCAN or DoIP transport
  -> POSIX adapters or typed STM32 BSP
  -> audited socket/MMIO boundary
```

F413 and G474 share `board_apps.rs`; only typed board tables, CAN/flash/UART
drivers, clock parameters, linker memory, and honest capability declarations
differ. Fixed queues and static protocol buffers make exhaustion observable.
No C++ OpenBSW module is linked into the Rust production graph.
