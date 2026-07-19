param(
    [Parameter(Mandatory = $true)]
    [string]$RootA,
    [Parameter(Mandatory = $true)]
    [string]$RootB,
    [Parameter(Mandatory = $true)]
    [string]$OutputRoot
)

$ErrorActionPreference = "Stop"
$workspace = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$privateRoot = [IO.Path]::GetFullPath((Join-Path $workspace "target/private-evidence"))
$output = [IO.Path]::GetFullPath((Join-Path $workspace $OutputRoot))
if (-not $output.StartsWith($privateRoot + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw "OutputRoot must be below target/private-evidence"
}
if (Test-Path -LiteralPath $output) {
    throw "OutputRoot already exists; release artifact sets are immutable"
}
$roots = @((Resolve-Path -LiteralPath $RootA).Path, (Resolve-Path -LiteralPath $RootB).Path)
foreach ($root in $roots) {
    if (-not (Test-Path -LiteralPath (Join-Path $root "Cargo.lock") -PathType Leaf)) {
        throw "each clean root must contain Cargo.lock"
    }
}

$artifacts = @(
    "app_f413", "app_g474", "blink_f413", "blink_g474",
    "can_server_f413", "can_server_g474", "nvm_test_f413", "nvm_test_g474",
    "can_stress_f413", "can_stress_g474", "can_busoff_f413", "can_busoff_g474",
    "safety_fault_f413", "safety_fault_g474"
)
$sets = @("a", "b")
New-Item -ItemType Directory -Path $output | Out-Null
foreach ($set in $sets) {
    New-Item -ItemType Directory -Path (Join-Path $output $set) | Out-Null
    New-Item -ItemType Directory -Path (Join-Path $output "maps-$set") | Out-Null
}
New-Item -ItemType Directory -Path (Join-Path $output "selected") | Out-Null

$records = @()
for ($index = 0; $index -lt $roots.Count; $index += 1) {
    $root = $roots[$index]
    $set = $sets[$index]
    foreach ($name in $artifacts) {
        $source = Join-Path $root "target/thumbv7em-none-eabihf/release/examples/$name"
        if (-not (Test-Path -LiteralPath $source -PathType Leaf)) {
            throw "missing clean-build artifact: $name"
        }
        $destination = Join-Path $output "$set/$name"
        & python -B (Join-Path $workspace "tools/port/finalize_rom_crc.py") `
            --input $source --output $destination
        if ($LASTEXITCODE -ne 0) { throw "ROM CRC finalization failed for $name" }
        $records += [ordered]@{
            set = $set
            name = $name
            bytes = (Get-Item -LiteralPath $destination).Length
            sha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $destination).Hash.ToLowerInvariant()
        }
    }
}

$comparisons = foreach ($name in $artifacts) {
    $left = $records | Where-Object { $_.set -eq "a" -and $_.name -eq $name }
    $right = $records | Where-Object { $_.set -eq "b" -and $_.name -eq $name }
    $identical = $left.bytes -eq $right.bytes -and $left.sha256 -eq $right.sha256
    if (-not $identical) { throw "independent finalized artifacts differ: $name" }
    Copy-Item -LiteralPath (Join-Path $output "a/$name") -Destination (Join-Path $output "selected/$name")
    [ordered]@{ name = $name; bytes = $left.bytes; identical = $identical; sha256 = $left.sha256 }
}

$sizeTool = @("arm-none-eabi-size", "llvm-size", "rust-size") |
    Where-Object { Get-Command $_ -ErrorAction SilentlyContinue } |
    Select-Object -First 1
if (-not $sizeTool) { throw "an LLVM or Arm GNU size executable is required" }
$productionSizes = @()
foreach ($set in $sets) {
    foreach ($name in @("app_f413", "app_g474")) {
        $mapSource = Join-Path $roots[[Array]::IndexOf($sets, $set)] `
            "target/thumbv7em-none-eabihf/release/examples/$name.map"
        if (-not (Test-Path -LiteralPath $mapSource -PathType Leaf)) {
            throw "missing release map: $name ($set)"
        }
        Copy-Item -LiteralPath $mapSource -Destination (Join-Path $output "maps-$set/$name.map")
        $sizeOutput = (& $sizeTool (Join-Path $output "$set/$name") 2>&1 | Out-String).Trim()
        if ($LASTEXITCODE -ne 0) { throw "size inspection failed: $name ($set)" }
        $productionSizes += [ordered]@{ set = $set; name = $name; output = $sizeOutput }
    }
}

$document = [ordered]@{
    schema_version = 1
    policy = "independent-clean-root-byte-equality-after-rom-crc-finalization"
    artifact_count = $artifacts.Count
    all_identical = $true
    artifacts = @($comparisons)
    maps = "maps-a and maps-b"
    size_tool = $sizeTool
    production_sizes = @($productionSizes)
}
[IO.File]::WriteAllText(
    (Join-Path $output "private-artifact-manifest.json"),
    (($document | ConvertTo-Json -Depth 6) + [Environment]::NewLine),
    [Text.UTF8Encoding]::new($false)
)
Write-Output ([pscustomobject]@{
    schema_version = $document.schema_version
    policy = $document.policy
    artifact_count = $document.artifact_count
    all_identical = $document.all_identical
} | ConvertTo-Json)
