param(
    [int]$RosDomainId = 0,
    [string]$WslDistribution = "Ubuntu-22.04",
    [string]$WslRepoRoot = "~/workspace/Cub_ROS",
    [string]$RosInterfaceIp = ""
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$player = Join-Path $repoRoot "unity\CubSim\Builds\Windows\CubSim.exe"
$log = Join-Path $repoRoot "unity\CubSim\Builds\Windows\e2e-player.log"
if (-not (Test-Path $player)) { throw "CubSim.exe is missing. Run unity/scripts/build.ps1 first." }

$containerId = (wsl.exe -d $WslDistribution -- bash -lc `
    "cd $WslRepoRoot && ./docker/run_in_container.sh true >/dev/null && docker inspect cub_ros --format '{{.Id}}'").Trim()
if ($LASTEXITCODE -ne 0 -or -not $containerId) {
    throw "The normal cub_ros container could not be started in $WslDistribution."
}

$buildCommand = "cd $WslRepoRoot && ./docker/run_in_container.sh 'cbs cub_simulation'"
wsl.exe -d $WslDistribution -- bash -lc $buildCommand
if ($LASTEXITCODE -ne 0) { throw "Failed to build cub_simulation in the normal cub_ros container." }

$playerArguments = @(
    "-batchmode", "-nographics", "--auto-start", "--ros-domain-id", "$RosDomainId",
    "-logFile", $log
)
if ($RosInterfaceIp) {
    $playerArguments += @("--ros-interface-ip", $RosInterfaceIp)
}

$process = $null
try {
    $process = Start-Process $player -ArgumentList $playerArguments -PassThru
    Start-Sleep -Seconds 6
    if ($process.HasExited) { throw "CubSim exited before the ROS test started. See $log" }

    $listCommand = "cd $WslRepoRoot && ./docker/run_in_container.sh " +
        "'ROS_DOMAIN_ID=$RosDomainId ros2 topic list'"
    $topicList = wsl.exe -d $WslDistribution -- bash -lc $listCommand
    if ($LASTEXITCODE -ne 0) { throw "ros2 topic list failed through the normal cub_ros container." }
    $requiredTopics = @("/clock", "/cmd_vel_atom", "/odom", "/scan", "/tf")
    foreach ($topic in $requiredTopics) {
        if ($topicList -notcontains $topic) { throw "Unity topic is missing from ros2 topic list: $topic" }
    }
    Write-Host ($topicList -join [Environment]::NewLine)

    $probeCommand = "cd $WslRepoRoot && ./docker/run_in_container.sh " +
        "'ROS_DOMAIN_ID=$RosDomainId ros2 run cub_simulation cub_simulation_smoke --ros-args -p timeout:=30.0'"
    wsl.exe -d $WslDistribution -- bash -lc $probeCommand
    if ($LASTEXITCODE -ne 0) { throw "Unity-ROS 2 smoke test failed. See $log" }
    Write-Host "Unity-ROS 2 smoke test passed through the normal cub_ros container."
} finally {
    if ($process -and -not $process.HasExited) { Stop-Process -Id $process.Id -Force }
}
