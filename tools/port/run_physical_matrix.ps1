param(
    [Parameter(Mandatory = $true)]
    [string]$ArtifactRoot,
    [Parameter(Mandatory = $true)]
    [string]$OutputRoot
)

$ErrorActionPreference = "Stop"
$Workspace = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$Artifacts = (Resolve-Path -LiteralPath $ArtifactRoot).Path
$PrivateRoot = [IO.Path]::GetFullPath((Join-Path $Workspace "target/private-evidence"))
$Output = [IO.Path]::GetFullPath((Join-Path $Workspace $OutputRoot))
if (-not $Artifacts.StartsWith($PrivateRoot + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw "ArtifactRoot must be below target/private-evidence"
}
if (-not $Output.StartsWith($PrivateRoot + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw "OutputRoot must be below target/private-evidence"
}
if (Test-Path -LiteralPath $Output) {
    throw "OutputRoot already exists; physical matrix outputs are immutable"
}

$RequiredArtifacts = @(
    "app_f413", "app_g474", "blink_f413", "blink_g474",
    "can_server_f413", "can_server_g474", "nvm_test_f413", "nvm_test_g474",
    "can_stress_f413", "can_stress_g474", "can_busoff_f413", "can_busoff_g474",
    "safety_fault_f413", "safety_fault_g474"
)
foreach ($name in $RequiredArtifacts) {
    if (-not (Test-Path -LiteralPath (Join-Path $Artifacts $name) -PathType Leaf)) {
        throw "missing exact release artifact: $name"
    }
}

function Get-ProbeInventory {
    $raw = (& probe-rs list 2>&1 | Out-String)
    $selectors = @(
        [regex]::Matches($raw, '(?i)([0-9a-f]{4}:[0-9a-f]{4}:[^\s]+)') |
            ForEach-Object { $_.Groups[1].Value.Trim() } |
            Select-Object -Unique
    )
    $devices = @(Get-PnpDevice -PresentOnly)
    $ports = @(Get-PnpDevice -Class Ports -PresentOnly)
    $inventory = @()
    foreach ($selector in $selectors) {
        $parts = $selector -split ':', 3
        $serial = $parts[2]
        $debug = @(
            $devices |
                Where-Object { $_.InstanceId -match '^USB\\VID_0483&PID_374[BE]&MI_00\\' } |
                Where-Object {
                    $parent = (Get-PnpDeviceProperty -InstanceId $_.InstanceId `
                        -KeyName 'DEVPKEY_Device_Parent').Data
                    (($parent -split '\\')[-1]) -eq $serial
                }
        )
        if ($debug.Count -ne 1) { throw "probe-to-PnP debug mapping is not unique" }
        $container = (Get-PnpDeviceProperty -InstanceId $debug[0].InstanceId `
            -KeyName 'DEVPKEY_Device_ContainerId').Data
        $mappedPorts = @(
            $ports | Where-Object {
                (Get-PnpDeviceProperty -InstanceId $_.InstanceId `
                    -KeyName 'DEVPKEY_Device_ContainerId' -ErrorAction SilentlyContinue).Data -eq $container
            }
        )
        if ($mappedPorts.Count -ne 1 -or $mappedPorts[0].FriendlyName -notmatch '\((COM\d+)\)') {
            throw "probe-to-virtual-COM mapping is not unique"
        }
        $port = $Matches[1]
        $idOutput = & probe-rs read b32 0xE0042000 1 --chip STM32F413ZH `
            --probe $selector --non-interactive 2>$null
        if ($LASTEXITCODE -ne 0 -or $idOutput -notmatch '([0-9A-Fa-f]{8})') {
            $target = "other"
        }
        else {
            $deviceId = [Convert]::ToUInt32($Matches[1], 16) -band 0xfff
            $target = switch ($deviceId) {
                0x463 { "stm32f413" }
                0x469 { "stm32g474" }
                default { "other" }
            }
        }
        $inventory += [pscustomobject]@{
            Target = $target
            Selector = $selector
            Port = $port
        }
    }
    return $inventory
}

function Get-SocketCanFixtures {
    $config = Join-Path ([Environment]::GetFolderPath("UserProfile")) ".ssh/config"
    if (-not (Test-Path -LiteralPath $config)) { throw "SSH configuration is unavailable" }
    $aliases = @()
    foreach ($line in Get-Content -LiteralPath $config) {
        if ($line -match '^\s*Host\s+([^*?\s]+)\s*$') { $aliases += $Matches[1] }
    }
    $qualified = @()
    foreach ($alias in ($aliases | Select-Object -Unique)) {
        $savedPreference = $ErrorActionPreference
        $ErrorActionPreference = "Continue"
        try {
            $interfaces = @(& ssh -o BatchMode=yes -o ConnectTimeout=3 $alias `
                'for n in /sys/class/net/*; do if [ "$(cat "$n/type")" = 280 ] && [ -e "$n/device" ]; then basename "$n"; fi; done' `
                2>$null)
            $fixtureExit = $LASTEXITCODE
        }
        finally {
            $ErrorActionPreference = $savedPreference
        }
        if ($fixtureExit -ne 0 -or $interfaces.Count -ne 1 -or $interfaces[0] -notmatch '^[A-Za-z0-9_.-]{1,15}$') { continue }
        $interface = $interfaces[0]
        $pythonCommand = "python3 -c 'import can'"
        $ErrorActionPreference = "Continue"
        try {
            & ssh $alias $pythonCommand 2>$null
            $pythonExit = $LASTEXITCODE
        }
        finally {
            $ErrorActionPreference = $savedPreference
        }
        if ($pythonExit -ne 0) { continue }
        $ErrorActionPreference = "Continue"
        try {
            & ssh $alias "sudo -n ip link show $interface" *> $null
            $controlExit = $LASTEXITCODE
        }
        finally {
            $ErrorActionPreference = $savedPreference
        }
        if ($controlExit -eq 0) {
            $qualified += [pscustomobject]@{ Alias = $alias; Interface = $interface }
        }
    }
    if ($qualified.Count -eq 0) {
        throw "no controllable configured SocketCAN fixture was discovered"
    }
    return $qualified
}

function Invoke-ProbeDownload($Probe, [string]$Target, [string]$Artifact) {
    $chip = if ($Target -eq "stm32f413") { "STM32F413ZH" } else { "STM32G474RETx" }
    $savedPreference = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        & probe-rs download --chip $chip --probe $Probe.Selector --verify `
            --disable-progressbars $Artifact *> $null
        $downloadExit = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $savedPreference
    }
    if ($downloadExit -ne 0) { throw "verified probe download failed" }
}

function Reset-CanFixture($Fixture) {
    $interface = $Fixture.Interface
    & ssh $Fixture.Alias `
        "sudo -n ip link set $interface down; sudo -n ip link set $interface type can bitrate 500000; sudo -n ip link set $interface up" `
        *> $null
    if ($LASTEXITCODE -ne 0) { throw "SocketCAN fixture reset failed" }
}

$RemoteProbe = @'
import can,json,time
bus=can.Bus(interface="socketcan",channel="__CAN_INTERFACE__")
while bus.recv(0.0) is not None: pass
bus.send(can.Message(arbitration_id=0x600,data=[2,0x3e,0,0,0,0,0,0],is_extended_id=False))
positive=False
deadline=time.monotonic()+1.0
while time.monotonic()<deadline:
    message=bus.recv(0.05)
    if message is not None and message.arbitration_id==0x601 and bytes(message.data)[:3]==bytes([2,0x7e,0]):
        positive=True
        break
print(json.dumps({"positive":positive}))
bus.shutdown()
'@

$RuntimeProbe = @'
import json,subprocess,sys,time
state={"boot":False,"ready":False,"panic":False,"error":"none","phase":"setup"}
try:
    import serial
    chip,selector,port_name=sys.argv[2:5]
    state["phase"]="open"
    with serial.Serial(port_name,115200,timeout=0.05) as port:
        state["phase"]="purge"
        try:
            port.reset_input_buffer()
        except serial.SerialException:
            pass
        state["phase"]="reset"
        subprocess.run(["probe-rs","reset","--chip",chip,"--probe",selector],check=True,capture_output=True)
        data=bytearray()
        deadline=time.monotonic()+2.0
        state["phase"]="read"
        while time.monotonic()<deadline:
            data.extend(port.read(512))
    text=data.decode("utf-8",errors="replace")
    state.update(boot="openbsw role=CanServer" in text,ready="UDS/ISO-TP server" in text,panic="panicked at" in text)
except Exception as error:
    state["error"]=type(error).__name__
print(json.dumps(state))
'@

function Get-BoardRuntimeState($Probe, [string]$Target) {
    $chip = if ($Target -eq "stm32f413") { "STM32F413ZH" } else { "STM32G474RETx" }
    $encoded = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($RuntimeProbe))
    $raw = & python -B -c 'import base64,sys;exec(base64.b64decode(sys.argv[1]))' `
        $encoded $chip $Probe.Selector $Probe.Port 2>$null
    if ($LASTEXITCODE -ne 0) { throw "privacy-safe board runtime probe failed" }
    return $raw | ConvertFrom-Json
}

function Test-CanResponder($Probe, [string]$Target, $Fixture) {
    $role = if ($Target -eq "stm32f413") { "can_server_f413" } else { "can_server_g474" }
    Invoke-ProbeDownload $Probe $Target (Join-Path $Artifacts $role)
    $runtime = Get-BoardRuntimeState $Probe $Target
    Write-Host ("{0} runtime: boot={1}; ready={2}; panic={3}; error={4}; phase={5}" -f `
        $Target, $runtime.boot, $runtime.ready, $runtime.panic, $runtime.error, $runtime.phase)
    Reset-CanFixture $Fixture
    Start-Sleep -Milliseconds 250
    $script = $RemoteProbe.Replace("__CAN_INTERFACE__", $Fixture.Interface)
    $encoded = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($script))
    $raw = & ssh $Fixture.Alias "echo $encoded | base64 -d | python3" 2>$null
    if ($LASTEXITCODE -ne 0) { throw "SocketCAN responder probe failed" }
    return [bool](($raw | ConvertFrom-Json).positive)
}

function Set-PeerIsolation($Inventory, $Selected) {
    foreach ($probe in $Inventory) {
        if ($probe.Target -notin @("stm32f413", "stm32g474")) { continue }
        if ($null -ne $Selected -and $probe.Selector -eq $Selected.Selector) { continue }
        $blink = if ($probe.Target -eq "stm32f413") { "blink_f413" } else { "blink_g474" }
        Invoke-ProbeDownload $probe $probe.Target (Join-Path $Artifacts $blink)
    }
}

function Invoke-Hil([string]$Script, [string[]]$Arguments) {
    & python -B $Script @Arguments
    if ($LASTEXITCODE -ne 0) { throw "authoritative HIL command failed: $Script" }
}

$inventory = @(Get-ProbeInventory)
$f413 = @($inventory | Where-Object { $_.Target -eq "stm32f413" })
$g474 = @($inventory | Where-Object { $_.Target -eq "stm32g474" })
if ($f413.Count -ne 1 -or $g474.Count -lt 2) {
    throw "expected one F413 and at least two G474 targets"
}
$fixtures = @(Get-SocketCanFixtures)
$resolved = @()
$fixtureOrdinal = 0
foreach ($fixtureCandidate in $fixtures) {
    $fixtureOrdinal += 1
    Set-PeerIsolation $inventory $null
    $f413Responders = @($f413 | Where-Object { Test-CanResponder $_ "stm32f413" $fixtureCandidate })
    Set-PeerIsolation $inventory $null
    $g474Responders = @($g474 | Where-Object {
        Set-PeerIsolation $inventory $_
        Test-CanResponder $_ "stm32g474" $fixtureCandidate
    })
    Write-Host ("candidate {0}: F413 responders={1}; G474 responders={2}" -f `
        $fixtureOrdinal, $f413Responders.Count, $g474Responders.Count)
    if ($f413Responders.Count -eq 1 -and $g474Responders.Count -eq 1) {
        $resolved += [pscustomobject]@{
            Fixture = $fixtureCandidate
            F413 = $f413Responders[0]
            G474 = $g474Responders[0]
        }
    }
}
if ($resolved.Count -ne 1) {
    throw "physical fixture and duplicate-G474 selection did not resolve to exactly one topology"
}
$fixture = $resolved[0].Fixture
$selectedF413 = $resolved[0].F413
$selectedG474 = $resolved[0].G474

$env:OPENBSW_HIL_FIXTURE_ID = "fixture-primary"
$env:OPENBSW_HIL_SSH = $fixture.Alias
$env:OPENBSW_HIL_CAN_INTERFACE = $fixture.Interface
$env:OPENBSW_HIL_F413_PROBE = $selectedF413.Selector
$env:OPENBSW_HIL_F413_SERIAL = $selectedF413.Port
$env:OPENBSW_HIL_G474_PROBE = $selectedG474.Selector
$env:OPENBSW_HIL_G474_SERIAL = $selectedG474.Port
$env:OPENBSW_HIL_PEER_ISOLATED = "1"
New-Item -ItemType Directory -Path $Output | Out-Null

try {
    foreach ($target in @("stm32f413", "stm32g474")) {
        $selected = if ($target -eq "stm32f413") { $selectedF413 } else { $selectedG474 }
        $suffix = if ($target -eq "stm32f413") { "f413" } else { "g474" }
        Set-PeerIsolation $inventory $selected
        Invoke-Hil "hil/deterministic_fixture.py" @(
            "--board", $target, "--artifact", (Join-Path $Artifacts "app_$suffix"),
            "--fixture-id", "fixture-primary", "--runs", "2",
            "--output", (Join-Path $Output "smoke-$suffix.json")
        )
        Set-PeerIsolation $inventory $selected
        Invoke-Hil "hil/storage_campaign.py" @(
            "--board", $target, "--artifact", (Join-Path $Artifacts "nvm_test_$suffix"),
            "--fixture-id", "fixture-primary", "--output", (Join-Path $Output "storage-$suffix.json")
        )
        Set-PeerIsolation $inventory $selected
        Invoke-Hil "hil/safety_campaign.py" @(
            "--board", $target, "--artifact", (Join-Path $Artifacts "safety_fault_$suffix"),
            "--fixture-id", "fixture-primary", "--output", (Join-Path $Output "safety-$suffix.json")
        )
        foreach ($mode in @("overflow", "busoff")) {
            $role = if ($mode -eq "overflow") { "can_stress_$suffix" } else { "can_busoff_$suffix" }
            Set-PeerIsolation $inventory $selected
            Invoke-Hil "hil/controller_fault_campaign.py" @(
                "--board", $target, "--mode", $mode, "--artifact", (Join-Path $Artifacts $role),
                "--fixture-id", "fixture-primary", "--output", (Join-Path $Output "$mode-$suffix.json")
            )
        }
        Set-PeerIsolation $inventory $selected
        Invoke-Hil "hil/stress_campaign.py" @(
            "--board", $target, "--artifact", (Join-Path $Artifacts "app_$suffix"),
            "--fixture-id", "fixture-primary", "--soak-seconds", "600", "--reset-count", "10",
            "--output", (Join-Path $Output "stress-$suffix.json")
        )
    }
}
finally {
    Set-PeerIsolation $inventory $null
    Remove-Item Env:OPENBSW_HIL_FIXTURE_ID -ErrorAction SilentlyContinue
    Remove-Item Env:OPENBSW_HIL_SSH -ErrorAction SilentlyContinue
    Remove-Item Env:OPENBSW_HIL_CAN_INTERFACE -ErrorAction SilentlyContinue
    Remove-Item Env:OPENBSW_HIL_F413_PROBE -ErrorAction SilentlyContinue
    Remove-Item Env:OPENBSW_HIL_F413_SERIAL -ErrorAction SilentlyContinue
    Remove-Item Env:OPENBSW_HIL_G474_PROBE -ErrorAction SilentlyContinue
    Remove-Item Env:OPENBSW_HIL_G474_SERIAL -ErrorAction SilentlyContinue
    Remove-Item Env:OPENBSW_HIL_PEER_ISOLATED -ErrorAction SilentlyContinue
}

Write-Output '{"boards":2,"duplicate_g474_resolved":true,"outcome":"passed"}'
