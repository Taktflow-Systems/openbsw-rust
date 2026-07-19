#!/usr/bin/env bash
set -euo pipefail

nightly=nightly-2026-03-14
rustup toolchain install "$nightly" --profile minimal
rustup component add rust-src --toolchain "$nightly"
cargo "+$nightly" install cargo-fuzz --version 0.13.2 --locked

targets=(
  can_frame
  com_packing
  docan_state
  doip_diagnostic
  doip_entity
  doip_header
  doip_payload
  doip_tcp
  fixed_containers
  isotp_codec
  memory_queue
  storage_record
  uds_connections
  uds_dispatch
)

for target in "${targets[@]}"; do
  echo "FUZZ_TARGET_BEGIN $target"
  cargo "+$nightly" fuzz run "$target" -- -runs=100
  echo "FUZZ_TARGET_PASS $target"
done
