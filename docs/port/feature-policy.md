# Cargo feature policy

Portable crates default to `std` for host ergonomics and must compile with
`--no-default-features` for embedded use. Features are additive except for MCU
selection. `bsw-can/can-fd` changes frame capacity from 8 to 64 bytes and must
work independently of `std`.

`bsw-bsp-stm32` requires exactly one of `stm32f413` or `stm32g474` for ARM
builds. Enabling both is always an error. Enabling neither remains permitted on
the host so platform-independent BSP queue tests can run.

`tools/port/check_features.py` is the executable matrix. It checks classic CAN
and CAN FD with and without `std`, both valid MCU targets, and the two invalid
MCU selections. An invalid combination must fail with the explicit
"exactly one MCU feature" diagnostic.
