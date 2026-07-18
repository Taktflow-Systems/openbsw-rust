[CmdletBinding()]
param([switch]$IncludeHilCollection)

$ErrorActionPreference = 'Continue'
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
Push-Location $RepoRoot
try {
    $Failures = 0
    function Invoke-Step([string]$Name, [scriptblock]$Command) {
        Write-Output "`n=== $Name ==="
        & $Command
        if ($LASTEXITCODE -ne 0) {
            $script:Failures++
            Write-Error "$Name failed with exit code $LASTEXITCODE"
        }
    }

    rustc --version --verbose
    cargo --version
    python --version

    Invoke-Step 'host tests' { cargo test --workspace --exclude bsw-bsp-stm32 }
    $TestList = cargo test --workspace --exclude bsw-bsp-stm32 -- --list 2>$null
    $TestEntries = @($TestList | Where-Object { $_ -match ': test$' }).Count
    Write-Output "Rust test entries: $TestEntries (inspect test output for ignored doctests)"
    Invoke-Step 'all portable features' { cargo check --workspace --exclude bsw-bsp-stm32 --all-features }
    Invoke-Step 'clippy' { cargo clippy --workspace --exclude bsw-bsp-stm32 --all-targets --all-features -- -D warnings }
    Invoke-Step 'feature matrix' { python tools/port/check_features.py }
    Invoke-Step 'no_std and allocation' { python tools/port/check_no_std.py }
    Invoke-Step 'F4 release example' { cargo build -p bsw-bsp-stm32 --release --example bsw_stack_f413 --no-default-features --features stm32f413 --target thumbv7em-none-eabihf }
    Invoke-Step 'G4 release example' { cargo build -p bsw-bsp-stm32 --release --example bsw_stack_g474 --no-default-features --features stm32g474 --target thumbv7em-none-eabihf }
    Invoke-Step 'release sizes' { arm-none-eabi-size target/thumbv7em-none-eabihf/release/examples/bsw_stack_f413 target/thumbv7em-none-eabihf/release/examples/bsw_stack_g474 }
    if ($IncludeHilCollection) {
        Invoke-Step 'HIL collection' { python -m pytest hil --collect-only -q }
    }
    if ($Failures -ne 0) { exit 1 }
} finally {
    Pop-Location
}
