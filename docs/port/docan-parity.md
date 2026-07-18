# DoCAN parity map - packages E09-E13

Pinned upstream: `ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77`, module
`libs/bsw/docan`.

| Behavior | Rust implementation | Evidence |
|---|---|---|
| Normal, extended, and mixed addressing | `bsw_docan::addressing::Addressing` | `docan_complete::normal_extended_and_mixed_codec_vectors_cover_classic_layouts` |
| Classic CAN and CAN FD PCI lengths | `bsw_docan::codec` and `ConnectionConfig::frame_size` | codec boundary suite; `can-fd` feature build |
| RX allocation retry, WAIT/OVFLW, block size, sequence wrap | `bsw_docan::transport::RxSession` | injected-clock RX tests and 125-byte wrap test |
| N_Br and N_Cr | `RxSession::poll` | exact-boundary tests in `docan_complete` |
| N_As, N_Bs, N_Cs, STmin, WAIT limit, cancellation | `bsw_docan::transport::TxSession` | exact-boundary and recovery tests in `docan_complete` |
| Platform-independent transport and pools | `RxSession`, `TxSession`, `TransportMessage` | in-process `bsw_can::virtual_bus` 125-byte transfer |
| Differential state trace | A05/A07 JSON oracle protocol | `docan-openbsw.json` versus `docan-rust.json` |
| Malformed codec and transition fuzzing | `isotp_codec`, `docan_state` | mandatory fuzz smoke targets |

The Rust port uses explicit generation-checked transport ownership and injected
monotonic instants instead of upstream intrusive lists and async timer handles.
Those are native replacements; frame bytes, timer boundaries, flow-control
semantics, sequence wrapping, callback results, and recovery behavior are the
parity contract.
