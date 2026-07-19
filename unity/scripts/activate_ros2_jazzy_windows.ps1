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

$vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) {
    throw "Visual Studio Installer (vswhere.exe) was not found. Install Visual Studio 2022 with the Desktop development with C++ workload."
}
$vsInstall = (& $vswhere -latest -products * -property installationPath | Select-Object -First 1)
if (-not $vsInstall) {
    throw "Visual Studio was not found. Install Visual Studio 2022 with the Desktop development with C++ workload."
}
$vsDevCmd = Join-Path $vsInstall "Common7\Tools\VsDevCmd.bat"
if (-not (Test-Path $vsDevCmd)) { throw "VsDevCmd.bat was not found under $vsInstall." }

# Import the x64 MSVC environment into this PowerShell.  Launch-VsDevShell.ps1
# can fail on localized Visual Studio metadata, so use the batch entry point.
$devCommand = '"' + $vsDevCmd + '" -no_logo -arch=x64 -host_arch=x64 && set'
$devEnvironment = & $env:ComSpec /d /s /c $devCommand
if ($LASTEXITCODE -ne 0) { throw "Visual Studio x64 developer environment activation failed." }
$developerPath = $devEnvironment | Where-Object { $_ -cmatch '^PATH=' } | Select-Object -First 1
foreach ($line in $devEnvironment) {
    if ($line -notmatch '^([^=]+)=(.*)$') { continue }
    $name = $Matches[1]
    if ($name -ieq "PATH") { continue }
    Set-Item -Path "Env:$name" -Value $Matches[2]
}
if (-not $developerPath) { throw "Visual Studio developer PATH was not returned by VsDevCmd.bat." }
$env:Path = $developerPath.Substring(5)

if (-not (Get-Command cl.exe -ErrorAction SilentlyContinue)) {
    throw "MSVC cl.exe was not found. Add the Desktop development with C++ workload and a Windows SDK in Visual Studio Installer."
}
if (-not (Get-Command ninja.exe -ErrorAction SilentlyContinue)) {
    throw "Ninja was not found in the pixi environment. Rerun install_ros2_jazzy_windows.ps1 without -CheckOnly."
}
Write-Host "ROS 2 $env:ROS_DISTRO activated from $rosSetup"
