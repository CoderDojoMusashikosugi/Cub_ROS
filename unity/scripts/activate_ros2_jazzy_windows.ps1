param(
    [string]$InstallRoot = "C:\pixi_ws"
)

$ErrorActionPreference = "Stop"
$InstallRoot = [IO.Path]::GetFullPath($InstallRoot).TrimEnd('\')
$pixi = Join-Path $InstallRoot "pixi.exe"
$pixiManifest = Join-Path $InstallRoot "pixi.toml"
$rosSetupCandidates = @(
    (Join-Path $InstallRoot "ros2-windows\local_setup.ps1"),
    (Join-Path $InstallRoot "ros2-windows\ros2-windows\local_setup.ps1")
)
$rosSetup = $rosSetupCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1

foreach ($required in @($pixi, $pixiManifest, $rosSetup)) {
    if (-not $required -or -not (Test-Path $required)) {
        throw "Windows ROS 2 Jazzy is not installed under $InstallRoot. Run unity\scripts\install_ros2_jazzy_windows.ps1 first."
    }
}

$hook = & $pixi shell-hook --manifest-path $pixiManifest --shell powershell
if ($LASTEXITCODE -ne 0) { throw "pixi environment activation failed." }
Invoke-Expression ($hook -join [Environment]::NewLine)
$env:QT_QPA_PLATFORM_PLUGIN_PATH = Join-Path $InstallRoot ".pixi\envs\default\Library\plugins\platforms"
. $rosSetup

if ($env:ROS_DISTRO -ne "jazzy") {
    throw "Expected ROS_DISTRO=jazzy, got '$env:ROS_DISTRO'."
}
Write-Host "ROS 2 $env:ROS_DISTRO activated from $rosSetup"
