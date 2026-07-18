[CmdletBinding()]
param(
    [ValidateSet('Checkout', 'Configure', 'Build', 'Test', 'Reference', 'All')]
    [string]$Action = 'Test',
    [string]$Distribution = 'Ubuntu-24.04',
    [ValidateRange(1, 32)]
    [int]$Jobs = 4
)

$ErrorActionPreference = 'Stop'
$PinnedCommit = 'ddbcf88a62dfcddb1eb07f868ba6412bec1ebf77'
$Repository = 'https://github.com/eclipse-openbsw/openbsw.git'
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$Checkout = Join-Path $RepoRoot 'target\oracle\openbsw'

if (-not (Test-Path $Checkout)) {
    New-Item -ItemType Directory -Force (Split-Path $Checkout) | Out-Null
    git clone --filter=blob:none --no-checkout $Repository $Checkout
    if ($LASTEXITCODE -ne 0) { throw 'OpenBSW clone failed' }
}

$Current = (git -C $Checkout rev-parse HEAD 2>$null)
if ($Current -ne $PinnedCommit) {
    git -C $Checkout fetch origin $PinnedCommit
    if ($LASTEXITCODE -ne 0) { throw 'OpenBSW commit fetch failed' }
    git -C $Checkout checkout --detach $PinnedCommit
    if ($LASTEXITCODE -ne 0) { throw 'OpenBSW checkout failed' }
}

$Verified = (git -C $Checkout rev-parse HEAD)
if ($Verified -ne $PinnedCommit) { throw "Oracle HEAD $Verified is not pinned commit $PinnedCommit" }
if ($Action -eq 'Checkout') { Write-Output "Pinned OpenBSW checkout ready: $Checkout"; exit 0 }

$Drive = $Checkout.Substring(0, 1).ToLowerInvariant()
$Rest = $Checkout.Substring(3).Replace('\', '/')
$WslCheckout = "/mnt/$Drive/$Rest"
if ($WslCheckout.Contains("'")) { throw 'Checkout path may not contain a single quote' }
$WslMirror = "/tmp/openbsw-rust-oracle-$($PinnedCommit.Substring(0, 12))"

# Compiling a C++ tree directly below /mnt is prohibitively slow. Keep the
# authoritative ignored checkout in the repository, but build an exact,
# disposable mirror on WSL's native filesystem.
wsl -d $Distribution -- bash -lc "set -euo pipefail; if [ ! -d '$WslMirror/.git' ]; then git clone --no-checkout '$WslCheckout' '$WslMirror'; fi; git -C '$WslMirror' checkout --detach '$PinnedCommit'; git -C '$WslMirror' diff --quiet; git -C '$WslMirror' diff --cached --quiet"
if ($LASTEXITCODE -ne 0) { throw 'OpenBSW WSL build mirror preparation failed' }

function Invoke-Oracle([string]$Command) {
    wsl -d $Distribution -- bash -lc "set -euo pipefail; cd '$WslMirror'; $Command"
    if ($LASTEXITCODE -ne 0) { throw "Oracle command failed: $Command" }
}

if ($Action -in @('Configure', 'Build', 'Test', 'All')) {
    Invoke-Oracle 'cmake --preset tests-posix-debug'
}
if ($Action -in @('Build', 'Test', 'All')) {
    Invoke-Oracle "cmake --build --preset tests-posix-debug --parallel $Jobs"
}
if ($Action -in @('Test', 'All')) {
    Invoke-Oracle 'ctest --preset tests-posix-debug --output-on-failure'
}
if ($Action -in @('Reference', 'All')) {
    Invoke-Oracle 'cmake --preset posix-freertos'
    Invoke-Oracle "cmake --build --preset posix-freertos --parallel $Jobs"
    # Keep `$?` escaped through WSL's outer command parser so the inner shell
    # evaluates the reference process exit status after `timeout` runs.
    Invoke-Oracle 'timeout --signal=TERM 5s build/posix-freertos/executables/referenceApp/application/Release/app.referenceApp.elf </dev/null || test \$? -eq 124'
}
