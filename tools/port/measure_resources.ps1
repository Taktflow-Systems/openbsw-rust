param(
    [switch]$Check,
    [string]$Output = "target/resource-baseline.json"
)

$ErrorActionPreference = "Stop"

function Invoke-Checked {
    param([string]$Program, [string[]]$Arguments)
    & $Program @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed: $Program $($Arguments -join ' ')"
    }
}

function Get-LlvmTool {
    param([string]$Name)
    $sysroot = (& rustc --print sysroot).Trim()
    $hostLine = & rustc -vV | Select-String '^host:'
    $hostTriple = ($hostLine -split ':', 2)[1].Trim()
    $extension = if ($IsWindows -or $env:OS -eq "Windows_NT") { ".exe" } else { "" }
    $path = Join-Path $sysroot "lib/rustlib/$hostTriple/bin/$Name$extension"
    if (-not (Test-Path -LiteralPath $path)) {
        throw "$Name is unavailable; install the llvm-tools component for the active toolchain"
    }
    return $path
}

function Get-ElfResources {
    param([string]$Path, [string]$LlvmSize, [string]$LlvmReadObj)
    $sizeLines = & $LlvmSize --format=berkeley $Path
    if ($LASTEXITCODE -ne 0) { throw "llvm-size failed for $Path" }
    $columns = ($sizeLines | Select-Object -Last 1).Trim() -split '\s+'
    $stackLines = & $LlvmReadObj --stack-sizes $Path
    if ($LASTEXITCODE -ne 0) { throw "llvm-readobj failed for $Path" }
    $stackSizes = $stackLines | Select-String 'Size: 0x([0-9A-Fa-f]+)' | ForEach-Object {
        [Convert]::ToInt64($_.Matches[0].Groups[1].Value, 16)
    }
    if (-not $stackSizes) { throw "No stack-size records found in $Path" }
    return [ordered]@{
        text_bytes = [int64]$columns[0]
        data_bytes = [int64]$columns[1]
        bss_bytes = [int64]$columns[2]
        flash_bytes = [int64]$columns[0] + [int64]$columns[1]
        largest_frame_bytes = [int64](($stackSizes | Measure-Object -Maximum).Maximum)
        stack_record_count = @($stackSizes).Count
    }
}

$builds = @(
    @{ example = "bsw_stack_f413"; feature = "stm32f413"; key = "stm32f413" },
    @{ example = "bsw_stack_g474"; feature = "stm32g474"; key = "stm32g474" }
)
foreach ($build in $builds) {
    Invoke-Checked cargo @(
        "+nightly", "rustc", "-p", "bsw-bsp-stm32", "--example", $build.example,
        "--no-default-features", "--features", $build.feature,
        "--target", "thumbv7em-none-eabihf", "--release", "--", "-Z", "emit-stack-sizes"
    )
}

$llvmSize = Get-LlvmTool "llvm-size"
$llvmReadObj = Get-LlvmTool "llvm-readobj"
$targets = [ordered]@{}
foreach ($build in $builds) {
    $elf = "target/thumbv7em-none-eabihf/release/examples/$($build.example)"
    $targets[$build.key] = Get-ElfResources $elf $llvmSize $llvmReadObj
}

$probeLines = & cargo run --quiet --release --manifest-path tools/resource-probe/Cargo.toml
if ($LASTEXITCODE -ne 0) { throw "Host resource probe failed" }
$probe = ($probeLines | Select-Object -Last 1) | ConvertFrom-Json
$result = [ordered]@{
    schema_version = 1
    target = $targets
    buffers = $probe.buffers
    benchmarks = $probe.benchmarks
}

$outputDirectory = Split-Path -Parent $Output
if ($outputDirectory) { New-Item -ItemType Directory -Force $outputDirectory | Out-Null }
$result | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $Output -Encoding utf8

if ($Check) {
    $limits = Get-Content -Raw docs/port/resource-limits.json | ConvertFrom-Json
    foreach ($key in @("stm32f413", "stm32g474")) {
        $actual = $result.target[$key]
        $limit = $limits.targets.$key
        if ($actual.flash_bytes -gt $limit.flash_bytes_max) { throw "$key flash regression" }
        if ($actual.bss_bytes -gt $limit.bss_bytes_max) { throw "$key BSS regression" }
        if ($actual.largest_frame_bytes -gt $limit.largest_frame_bytes_max) { throw "$key stack-frame regression" }
    }
    foreach ($name in $probe.buffers.PSObject.Properties.Name) {
        $maximumName = "${name}_max"
        if ($probe.buffers.$name -gt $limits.buffers.$maximumName) { throw "$name buffer regression" }
    }
    if ($probe.benchmarks.crc32_bytes_per_second -lt $limits.benchmarks.crc32_bytes_per_second_min) {
        throw "CRC32 host benchmark regression"
    }
    if ($probe.benchmarks.disabled_log_ops_per_second -lt $limits.benchmarks.disabled_log_ops_per_second_min) {
        throw "disabled-logging benchmark regression"
    }
    if ($probe.benchmarks.fixed_vec_ops_per_second -lt $limits.benchmarks.fixed_vec_ops_per_second_min) {
        throw "FixedVec host benchmark regression"
    }
}

$result | ConvertTo-Json -Depth 6
