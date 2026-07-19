param(
    [Parameter(Mandatory = $true)]
    [string]$BaseRevision,
    [Parameter(Mandatory = $true)]
    [string]$OutputRoot
)

$ErrorActionPreference = "Stop"
$workspace = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$output = [IO.Path]::GetFullPath((Join-Path $workspace $OutputRoot))
$privateRoot = [IO.Path]::GetFullPath((Join-Path $workspace "target/private-evidence"))
if (-not $output.StartsWith($privateRoot + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw "OutputRoot must be below target/private-evidence"
}
if (Test-Path -LiteralPath $output) {
    throw "OutputRoot already exists; clean roots are immutable"
}
if ((& git -C $workspace rev-parse $BaseRevision).Trim() -ne $BaseRevision) {
    throw "BaseRevision must be an exact full revision"
}

New-Item -ItemType Directory -Path $output | Out-Null
$patchPath = Join-Path $output "source.patch"
& git -C $workspace diff --binary --full-index --no-ext-diff "--output=$patchPath" $BaseRevision --
if ($LASTEXITCODE -ne 0) { throw "git diff failed" }
$newline = [Environment]::NewLine

$untracked = @(& git -C $workspace ls-files --others --exclude-standard) | Sort-Object
if ($LASTEXITCODE -ne 0) { throw "untracked source inventory failed" }
$untrackedEntries = foreach ($relative in $untracked) {
    $source = [IO.Path]::GetFullPath((Join-Path $workspace $relative))
    if (-not $source.StartsWith($workspace + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
        throw "untracked source escaped workspace"
    }
    [ordered]@{
        path = $relative.Replace("\", "/")
        size = (Get-Item -LiteralPath $source).Length
        sha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $source).Hash.ToLowerInvariant()
    }
}
$untrackedManifest = [ordered]@{
    schema_version = 1
    base_revision = $BaseRevision
    files = @($untrackedEntries)
}
[IO.File]::WriteAllText(
    (Join-Path $output "untracked-source-manifest.json"),
    (($untrackedManifest | ConvertTo-Json -Depth 6) + $newline),
    [Text.UTF8Encoding]::new($false)
)

$archive = Join-Path $output "base-source.tar"
& git -C $workspace archive --format=tar --output=$archive $BaseRevision -- `
    . `
    ":(exclude,glob)**/__pycache__/**" `
    ":(exclude,glob)**/.pytest_cache/**" `
    ":(exclude,glob)**/*.pyc" `
    ":(exclude,glob)**/*.pyo"
if ($LASTEXITCODE -ne 0) { throw "git archive failed" }

function New-CleanRoot {
    param([string]$Name)
    $root = Join-Path $output $Name
    New-Item -ItemType Directory -Path $root | Out-Null
    & tar -xf $archive -C $root
    if ($LASTEXITCODE -ne 0) { throw "base extraction failed for $Name" }
    if ((Get-Item -LiteralPath $patchPath).Length -gt 1) {
        # The private roots live below the enclosing worktree. Applying with
        # `-C $root` makes Git discover that parent worktree and skip every
        # patch as being outside its prefix. Apply from the workspace with an
        # explicit, validated directory prefix instead.
        if (-not $root.StartsWith($workspace + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
            throw "clean root escaped workspace"
        }
        $relativeRoot = $root.Substring($workspace.Length + 1).Replace("\", "/")
        & git -C $workspace apply --binary --whitespace=nowarn "--directory=$relativeRoot" $patchPath
        if ($LASTEXITCODE -ne 0) { throw "source patch application failed for $Name" }
        & git -C $workspace apply --check --reverse --binary "--directory=$relativeRoot" $patchPath
        if ($LASTEXITCODE -ne 0) {
            throw "source patch verification failed for $Name"
        }
    }
    foreach ($relative in $untracked) {
        $source = [IO.Path]::GetFullPath((Join-Path $workspace $relative))
        $destination = [IO.Path]::GetFullPath((Join-Path $root $relative))
        if (-not $destination.StartsWith($root + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
            throw "untracked destination escaped clean root"
        }
        $parent = Split-Path -Parent $destination
        if ($parent) { New-Item -ItemType Directory -Force -Path $parent | Out-Null }
        Copy-Item -LiteralPath $source -Destination $destination
    }
    return $root
}

function Get-SourceIdentity {
    param([string]$Root)
    $entries = foreach ($file in Get-ChildItem -LiteralPath $Root -Recurse -File | Sort-Object FullName) {
        $relative = $file.FullName.Substring($Root.Length + 1).Replace("\", "/")
        [ordered]@{
            path = $relative
            size = $file.Length
            sha256 = (Get-FileHash -Algorithm SHA256 -LiteralPath $file.FullName).Hash.ToLowerInvariant()
        }
    }
    $canonical = [ordered]@{
        schema_version = 1
        base_revision = $BaseRevision
        files = @($entries)
    } | ConvertTo-Json -Depth 6 -Compress
    $bytes = [Text.Encoding]::UTF8.GetBytes($canonical)
    $hasher = [Security.Cryptography.SHA256]::Create()
    try { $identity = ([BitConverter]::ToString($hasher.ComputeHash($bytes))).Replace("-", "").ToLowerInvariant() }
    finally { $hasher.Dispose() }
    return [ordered]@{
        identity = $identity
        file_count = @($entries).Count
        manifest = $canonical | ConvertFrom-Json
    }
}

$rootA = New-CleanRoot "root-a"
$rootB = New-CleanRoot "root-b"
$identityA = Get-SourceIdentity $rootA
$identityB = Get-SourceIdentity $rootB
if ($identityA.identity -ne $identityB.identity) {
    throw "independent clean roots have different source identities"
}
$metadata = [ordered]@{
    schema_version = 1
    base_revision = $BaseRevision
    source_bundle_identity = $identityA.identity
    source_file_count = $identityA.file_count
    roots = @("root-a", "root-b")
    locked_dependency_inputs = @(
        "Cargo.lock",
        "fuzz/Cargo.lock",
        "tools/resource-probe/Cargo.lock"
    )
    composition = "base revision minus generated Python caches plus source.patch plus untracked-source-manifest.json"
    excluded_generated_inputs = @("Python bytecode", "Python cache directories")
}
[IO.File]::WriteAllText(
    (Join-Path $output "source-identity.json"),
    (($metadata | ConvertTo-Json -Depth 6) + $newline),
    [Text.UTF8Encoding]::new($false)
)
[IO.File]::WriteAllText(
    (Join-Path $output "source-file-manifest.json"),
    (($identityA.manifest | ConvertTo-Json -Depth 6) + $newline),
    [Text.UTF8Encoding]::new($false)
)
Write-Output ($metadata | ConvertTo-Json -Depth 6)
