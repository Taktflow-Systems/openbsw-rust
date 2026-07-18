# Project E2E extension contract - package E34

The implementation in `bsw-util::e2e` is a project-specific wire-format
extension. It is not AUTOSAR E2E Profile 1 and does not claim conformance to
any AUTOSAR E2E profile.

| Semantic | Project contract | Evidence |
|---|---|---|
| CRC | CRC-8/SAE-J1850 by default over bytes `1..len` | fixed golden byte vectors and corruption tests in `bsw-util` |
| Counter | low nibble of byte 1, modulo 16; upper nibble preserved | wrap, repeat, gap, and upper-nibble tests |
| Data ID | unsupported: neither transmitted nor folded into the CRC | `E2eProfile::uses_data_id()` and golden vectors |
| Status | `Initial`, `Ok`, `Repeated`, `WrongCounter`, or `WrongCrc`; only initial/OK advance accepted receive state | checker state-transition tests |
| Buffer errors | checked APIs return `FrameTooShort` or `LengthExceedsBuffer` | no-panic boundary tests |
| COM integration | TX stamps the PDU shadow buffer; RX accepts only initial/OK frames and otherwise invalidates signals without update/deadline re-arm | `crates/bsw-com/tests/e2e_integration.rs` |

The two-byte overhead and reserved bit layout must be accounted for by COM
signal configuration. A future AUTOSAR profile implementation must use a
separate, profile-named type with its own Data ID modes and normative vectors;
changing this extension in place would silently change its wire contract.
