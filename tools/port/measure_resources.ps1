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
    $stackJson = & python tools/port/analyze_stack.py --readobj $LlvmReadObj --objdump arm-none-eabi-objdump --elf $Path
    if ($LASTEXITCODE -ne 0) { throw "stack analysis failed for $Path" }
    $stack = $stackJson | ConvertFrom-Json
    return [ordered]@{
        text_bytes = [int64]$columns[0]
        data_bytes = [int64]$columns[1]
        bss_bytes = [int64]$columns[2]
        flash_bytes = [int64]$columns[0] + [int64]$columns[1]
        ram_static_bytes = [int64]$columns[1] + [int64]$columns[2]
        noinit_reserved_bytes = 1024
        largest_frame_bytes = [int64]$stack.largest_frame_bytes
        stack_record_count = [int64]$stack.stack_record_count
        task_direct_path_bytes = [int64]$stack.task_direct_path_bytes
        indirect_call_reserve_bytes = [int64]$stack.indirect_call_reserve_bytes
        task_stack_required_bytes = [int64]$stack.task_stack_required_bytes
        isr_direct_path_bytes = [int64]$stack.isr_direct_path_bytes
        exception_frame_bytes = [int64]$stack.exception_frame_bytes
        isr_stack_increment_bytes = [int64]$stack.isr_stack_increment_bytes
        combined_msp_required_bytes = [int64]$stack.combined_msp_required_bytes
        total_ram_required_bytes = [int64]$columns[1] + [int64]$columns[2] + 1024 + [int64]$stack.combined_msp_required_bytes
        indirect_call_sites = [int64]$stack.indirect_call_sites
        cycle_edges_observed = [int64]$stack.cycle_edges_observed
    }
}

$builds = @(
    @{ example = "app_f413"; feature = "stm32f413"; key = "stm32f413" },
    @{ example = "app_g474"; feature = "stm32g474"; key = "stm32g474" }
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
    queues = $probe.queues
    timing = $probe.timing
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
        if ($actual.ram_static_bytes -gt $limit.ram_static_bytes_max) { throw "$key static-RAM regression" }
        if ($actual.noinit_reserved_bytes -gt $limit.noinit_reserved_bytes_max) { throw "$key NOINIT reservation regression" }
        if ($actual.total_ram_required_bytes -gt $limit.total_ram_required_bytes_max) { throw "$key total-RAM regression" }
        if ($actual.task_stack_required_bytes -gt $limit.task_stack_bytes_max) { throw "$key task-stack regression" }
        if ($actual.isr_stack_increment_bytes -gt $limit.isr_stack_increment_bytes_max) { throw "$key ISR-stack regression" }
        if ($actual.combined_msp_required_bytes -gt $limit.combined_msp_bytes_max) { throw "$key combined-MSP regression" }
        if ($actual.cycle_edges_observed -ne 0) { throw "$key recursive stack path requires review" }
    }
    foreach ($name in $probe.buffers.PSObject.Properties.Name) {
        $maximumName = $name + "_max"
        if ($null -eq $limits.buffers.$maximumName) { throw "undeclared buffer budget: $name" }
        if ($probe.buffers.$name -gt $limits.buffers.$maximumName) { throw "$name buffer regression" }
    }
    foreach ($name in $probe.queues.PSObject.Properties.Name) {
        if ($null -eq $limits.queues.$name) { throw "undeclared queue budget: $name" }
        if ($probe.queues.$name -gt $limits.queues.$name) { throw "$name queue budget regression" }
    }
    $timingMaximums = @{
        reference_cycle_us = "reference_cycle_us_max"
        heartbeat_period_us = "heartbeat_period_us_max"
        can_bus_off_recovery_ms = "can_bus_off_recovery_ms_max"
        uds_response_ms = "uds_response_ms_max"
        watchdog_fast_test_reset_ms = "watchdog_fast_test_reset_ms_max"
        watchdog_fast_test_deadline_ms = "watchdog_fast_test_deadline_ms_max"
        bounded_soak_minutes = "bounded_soak_minutes"
    }
    foreach ($name in $timingMaximums.Keys) {
        $limitName = $timingMaximums[$name]
        if ($probe.timing.$name -gt $limits.timing.$limitName) { throw "$name timing budget regression" }
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
