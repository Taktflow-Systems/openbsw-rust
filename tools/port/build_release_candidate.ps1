param(
    [Parameter(Mandatory = $true)]
    [string]$SourceRoot
)

$ErrorActionPreference = "Stop"
$root = (Resolve-Path -LiteralPath $SourceRoot).Path
if (-not (Test-Path -LiteralPath (Join-Path $root "Cargo.lock"))) {
    throw "SourceRoot does not contain Cargo.lock"
}

Push-Location $root
try {
    & cargo build --locked --release --manifest-path crates/openbsw-reference-app/Cargo.toml
    if ($LASTEXITCODE -ne 0) { throw "POSIX release build failed" }

    # Clean roots retained below the live workspace see both their own and the
    # ancestor .cargo/config.toml. Override the merged rustflags with the one
    # required linker-script argument so the invocation is location-stable.
    $previousEncodedFlags = $env:CARGO_ENCODED_RUSTFLAGS
    $env:CARGO_ENCODED_RUSTFLAGS = "-Clink-arg=-Tlink.x"
    try {
        & cargo build --locked --release --target thumbv7em-none-eabihf `
            -p bsw-bsp-stm32 --features stm32f413 --examples
        if ($LASTEXITCODE -ne 0) { throw "STM32F413 release build failed" }

        & cargo rustc --locked --release --target thumbv7em-none-eabihf `
            -p bsw-bsp-stm32 --features stm32f413 --example app_f413 -- `
            -C "link-arg=-Map=target/thumbv7em-none-eabihf/release/examples/app_f413.map"
        if ($LASTEXITCODE -ne 0) { throw "STM32F413 release map build failed" }

        & cargo build --locked --release --target thumbv7em-none-eabihf `
            -p bsw-bsp-stm32 --features stm32g474 --examples
        if ($LASTEXITCODE -ne 0) { throw "STM32G474 release build failed" }


        & cargo rustc --locked --release --target thumbv7em-none-eabihf `
            -p bsw-bsp-stm32 --features stm32g474 --example app_g474 -- `
            -C "link-arg=-Map=target/thumbv7em-none-eabihf/release/examples/app_g474.map"
        if ($LASTEXITCODE -ne 0) { throw "STM32G474 release map build failed" }
    }
    finally {
        if ($null -eq $previousEncodedFlags) {
            Remove-Item Env:CARGO_ENCODED_RUSTFLAGS -ErrorAction SilentlyContinue
        }
        else {
            $env:CARGO_ENCODED_RUSTFLAGS = $previousEncodedFlags
        }
    }
}
finally {
    Pop-Location
}
