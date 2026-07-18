# OpenBSW CRC oracle vectors

Source: pinned upstream files `libs/bsw/util/test/src/util/crc/Crc8Test.cpp`,
`Crc16Test.cpp`, `Crc32Test.cpp`, and `fixtures/crc/CrcTestFixture.h`.

Inputs are `31`, `00`, ASCII `123456789`, and nine zero bytes, in that order.

| Profile | Expected digests |
|---|---|
| CRC-8 CCITT | `97`, `00`, `F4`, `00` |
| OpenBSW CRC-8 SAEJ1850 | `57`, `00`, `37`, `00` |
| CRC-8 MAXIM | `E0`, `00`, `A1`, `00` |
| CRC-8 H2F | `4F`, `BD`, `DF`, `E1` |
| CRC-8 ROHC | `7A`, `CF`, `D0`, `F0` |
| CRC-16 CCITT | `C782`, `E1F0`, `29B1`, `1872` |
| CRC-32 Ethernet | `83DCEFB7`, `D202EF8D`, `CBF43926`, `E60914AE` |
| CRC-32 ARE2EP4 | `2DE7AF5E`, `6016DC99`, `1697D06A`, `4D43C7AA` |

The executable comparison is `crates/bsw-util/tests/crc_oracle.rs`.
The upstream SAEJ1850 parameters are exposed separately from the catalogue
CRC-8/SAE-J1850 profile used by the project-specific E2E extension.
