param(
    [string]$UnityEditor = "C:\Program Files\Unity\Hub\Editor\6000.4.2f1\Editor\Unity.exe",
    [switch]$SkipRos2AssetBuild
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$ros2ForUnity = Join-Path $repoRoot "unity\ros2-for-unity"
$assetSource = Join-Path $ros2ForUnity "install\asset\Ros2ForUnity"
$assetDestination = Join-Path $repoRoot "unity\CubSim\Assets\Ros2ForUnity"

& (Join-Path $PSScriptRoot "update_mcub_model.ps1") -UnityEditor $UnityEditor -SkipUnityValidation

if (-not (Test-Path $assetSource) -and -not $SkipRos2AssetBuild) {
    if ($env:ROS_DISTRO -ne "jazzy") {
        throw "Windows ROS 2 Jazzy is not sourced. Source local_setup.ps1, then rerun setup.ps1."
    }
    if (-not (Test-Path (Join-Path $ros2ForUnity "src\ros2cs"))) {
        Push-Location $ros2ForUnity
        try {
            & ".\pull_repositories.ps1"
            if ($LASTEXITCODE -ne 0) { throw "Failed to fetch ros2-for-unity dependencies." }
        } finally {
            Pop-Location
        }
    }
    & (Join-Path $PSScriptRoot "build_ros2_for_unity.ps1") -Clean
    if ($LASTEXITCODE -ne 0) { throw "ros2-for-unity standalone build failed." }
}

if (Test-Path $assetSource) {
    if (Test-Path $assetDestination) { Remove-Item -Recurse -Force $assetDestination }
    Copy-Item -Recurse -Force $assetSource $assetDestination
    $unity = Start-Process -FilePath $UnityEditor -ArgumentList @(
        "-batchmode", "-quit", "-projectPath", "$repoRoot\unity\CubSim",
        "-executeMethod", "CubSim.Editor.CubSimProjectTools.EnableRos2",
        "-logFile", "$repoRoot\unity\setup.log"
    ) -PassThru -Wait
    if ($unity.ExitCode -ne 0) { throw "Unity could not enable the ROS 2 integration. See unity/setup.log." }
    Write-Host "ROS 2 Unity Asset installed and CUB_ROS2 enabled."
} elseif ($SkipRos2AssetBuild) {
    Write-Warning "ROS 2 Asset installation was skipped; ROS topics remain disabled."
}
