param(
    [ValidateRange(1, 1000000)]
    [int]$Runs = 100
)

$ErrorActionPreference = "Stop"

# MSVC fuzz binaries load the AddressSanitizer runtime dynamically. Visual
# Studio installs it beside the compiler rather than on the default PATH.
if ($IsWindows -or $env:OS -eq "Windows_NT") {
    $programFilesX86 = [Environment]::GetFolderPath("ProgramFilesX86")
    $vswhere = Join-Path $programFilesX86 "Microsoft Visual Studio/Installer/vswhere.exe"
    if (-not (Test-Path -LiteralPath $vswhere)) {
        throw "Visual Studio 2022 Build Tools are required for MSVC fuzz execution"
    }
    $installation = (& $vswhere -latest -version "[17.0,18.0)" -products * `
        -property installationPath | Select-Object -First 1)
    if (-not $installation) {
        throw "Visual Studio 2022 Build Tools are required for MSVC fuzz execution; use Linux CI otherwise"
    }
    $asan = Get-ChildItem -LiteralPath $installation -Recurse `
        -Filter "clang_rt.asan_dynamic-x86_64.dll" -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if (-not $asan) {
        throw "The Visual Studio 2022 MSVC AddressSanitizer runtime is required"
    }
    $env:PATH = "$($asan.DirectoryName)$([IO.Path]::PathSeparator)$env:PATH"
    $env:LIB = "$($asan.DirectoryName)$([IO.Path]::PathSeparator)$env:LIB"
}

$targets = @(
    "can_frame",
    "isotp_codec",
    "uds_dispatch",
    "uds_connections",
    "doip_header",
    "doip_payload",
    "doip_tcp",
    "doip_diagnostic",
    "storage_record",
    "com_packing",
    "fixed_containers",
    "memory_queue",
    "docan_state"
)

foreach ($target in $targets) {
    & cargo +nightly fuzz run $target -- "-runs=$Runs"
    if ($LASTEXITCODE -ne 0) {
        throw "Fuzz smoke failed for $target"
    }
}
