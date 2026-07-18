param(
    [int]$RosDomainId = 0,
    [string]$RosInterfaceIp = ""
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$player = Join-Path $repoRoot "unity\CubSim\Builds\Windows\CubSim.exe"
if (-not (Test-Path $player)) { throw "CubSim.exe is missing. Run unity/scripts/build.ps1 first." }
$playerArguments = @("--ros-domain-id", "$RosDomainId")
if ($RosInterfaceIp) { $playerArguments += @("--ros-interface-ip", $RosInterfaceIp) }
& $player @playerArguments
