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
    throw "OutputRoot already exists; reproducibility results are immutable"
}
$roots = @((Resolve-Path -LiteralPath $RootA).Path, (Resolve-Path -LiteralPath $RootB).Path)
New-Item -ItemType Directory -Path $output | Out-Null

$records = @()
for ($index = 0; $index -lt $roots.Count; $index += 1) {
    $set = @("a", "b")[$index]
    $target = Join-Path $output "$set-target"
    New-Item -ItemType Directory -Path $target | Out-Null
    & docker run --rm `
        --mount "type=bind,source=$($roots[$index]),target=/work,readonly" `
        --mount "type=bind,source=$target,target=/out" `
        --workdir /work `
        --env CARGO_TARGET_DIR=/out `
        rust:1.94-bookworm `
        cargo build --locked --release --manifest-path crates/openbsw-reference-app/Cargo.toml
    if ($LASTEXITCODE -ne 0) { throw "pinned Linux POSIX build failed for set $set" }
    $artifact = Join-Path $target "release/openbsw-reference-app"
    if (-not (Test-Path -LiteralPath $artifact -PathType Leaf)) {
        throw "pinned Linux POSIX artifact is missing for set $set"
    }
    $records += [ordered]@{
        set = $set
        bytes = (Get-Item -LiteralPath $artifact).Length
        sha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $artifact).Hash.ToLowerInvariant()
    }
}
$identical = $records[0].bytes -eq $records[1].bytes -and $records[0].sha256 -eq $records[1].sha256
if (-not $identical) { throw "independent pinned Linux POSIX artifacts differ" }
Copy-Item -LiteralPath (Join-Path $output "a-target/release/openbsw-reference-app") `
    -Destination (Join-Path $output "selected-openbsw-reference-app")
$document = [ordered]@{
    schema_version = 1
    image = "rust:1.94-bookworm"
    source_mount = "/work"
    cargo_target_dir = "/out"
    bytes = $records[0].bytes
    sha256 = $records[0].sha256
    identical = $identical
}
[IO.File]::WriteAllText(
    (Join-Path $output "private-posix-reproducibility.json"),
    (($document | ConvertTo-Json) + [Environment]::NewLine),
    [Text.UTF8Encoding]::new($false)
)
Write-Output ([pscustomobject]@{
    schema_version = $document.schema_version
    image = $document.image
    source_mount = $document.source_mount
    cargo_target_dir = $document.cargo_target_dir
    bytes = $document.bytes
    identical = $document.identical
} | ConvertTo-Json)
