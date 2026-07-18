$ErrorActionPreference = "Stop"

cargo run --quiet -p bsw-middleware --bin bsw-middleware-codegen -- `
    crates/bsw-middleware/models/vehicle-control.mw `
    crates/bsw-middleware/src/generated/vehicle_control.rs `
    --check

if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
