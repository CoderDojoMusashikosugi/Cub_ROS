param(
    [string]$InstallRoot = "C:\pixi_ws",
    [string]$PixiVersion = "0.72.2",
    [string]$Ros2Release = "20250820",
    [switch]$CheckOnly
)

$ErrorActionPreference = "Stop"

if (-not $IsWindows -and $PSVersionTable.PSEdition -eq "Core") {
    throw "This script supports Windows only."
}
if (-not [Environment]::Is64BitOperatingSystem) {
    throw "ROS 2 Jazzy Windows binaries require 64-bit Windows."
}

$InstallRoot = [IO.Path]::GetFullPath($InstallRoot).TrimEnd('\')
if ($InstallRoot -ne "C:\pixi_ws") {
    Write-Warning "ROS 2 Windows binary archives are not fully relocatable. C:\pixi_ws is the supported install root."
}

$pixi = Join-Path $InstallRoot "pixi.exe"
$pixiManifest = Join-Path $InstallRoot "pixi.toml"
$pixiArchive = Join-Path $InstallRoot "pixi-$PixiVersion-windows-x64.zip"
$rosArchive = Join-Path $InstallRoot "ros2-jazzy-$Ros2Release-windows-release-amd64.zip"
$pixiUrl = "https://github.com/prefix-dev/pixi/releases/download/v$PixiVersion/pixi-x86_64-pc-windows-msvc.zip"
$pixiManifestUrl = "https://raw.githubusercontent.com/ros2/ros2/refs/heads/jazzy/pixi.toml"
$rosArchiveUrl = "https://github.com/ros2/ros2/releases/download/release-jazzy-$Ros2Release/ros2-jazzy-$Ros2Release-windows-release-amd64.zip"
$pixiSha256 = if ($PixiVersion -eq "0.72.2") { "6c6c4b1e7e55ec590bd755ae720956b5549526442f0dcb8707941d3289e31477" } else { $null }
$rosSha256 = if ($Ros2Release -eq "20250820") { "b2decbc86bdc6103fa839aba49dd320ad7a2e00657e97e8e2778ced0106a58c8" } else { $null }

function Find-RosSetup {
    $candidates = @(
        (Join-Path $InstallRoot "ros2-windows\local_setup.ps1"),
        # Accept the directory layout created by an older version of this setup.
        (Join-Path $InstallRoot "ros2-windows\ros2-windows\local_setup.ps1")
    )
    foreach ($candidate in $candidates) {
        if (Test-Path $candidate) { return $candidate }
    }
    return $null
}

function Download-File {
    param(
        [Parameter(Mandatory = $true)][string]$Uri,
        [Parameter(Mandatory = $true)][string]$Destination,
        [long]$MinimumBytes = 1,
        [string]$ExpectedSha256
    )

    if (Test-Path $Destination) {
        if ((Get-Item $Destination).Length -lt $MinimumBytes) {
            throw "Existing download is incomplete: $Destination. Remove it and rerun this script."
        }
        if ($ExpectedSha256) {
            $actual = (Get-FileHash -LiteralPath $Destination -Algorithm SHA256).Hash.ToLowerInvariant()
            if ($actual -ne $ExpectedSha256) { throw "SHA-256 mismatch: $Destination" }
        }
        Write-Host "Using existing download: $Destination"
        return
    }

    $partial = "$Destination.partial"
    if (Test-Path $partial) { Remove-Item -LiteralPath $partial -Force }
    Write-Host "Downloading $Uri"
    $curl = Get-Command curl.exe -ErrorAction SilentlyContinue
    if ($curl) {
        & $curl.Source -fL --retry 3 --output $partial $Uri
        if ($LASTEXITCODE -ne 0) { throw "Download failed: $Uri" }
    } else {
        Invoke-WebRequest -UseBasicParsing -Uri $Uri -OutFile $partial
    }
    if ((Get-Item $partial).Length -lt $MinimumBytes) {
        throw "Downloaded file is unexpectedly small: $Uri"
    }
    if ($ExpectedSha256) {
        $actual = (Get-FileHash -LiteralPath $partial -Algorithm SHA256).Hash.ToLowerInvariant()
        if ($actual -ne $ExpectedSha256) { throw "SHA-256 mismatch: $Uri" }
    } else {
        Write-Warning "No pinned SHA-256 is available for this overridden version: $Uri"
    }
    Move-Item -LiteralPath $partial -Destination $Destination
}

function Test-Environment {
    $rosSetup = Find-RosSetup
    foreach ($required in @($pixi, $pixiManifest, $rosSetup)) {
        if (-not $required -or -not (Test-Path $required)) {
            throw "Windows ROS 2 Jazzy environment is incomplete under $InstallRoot."
        }
    }

    & $pixi --version
    if ($LASTEXITCODE -ne 0) { throw "pixi could not be executed." }

    $hook = & $pixi shell-hook --manifest-path $pixiManifest --shell powershell
    if ($LASTEXITCODE -ne 0) { throw "pixi environment activation failed." }
    Invoke-Expression ($hook -join [Environment]::NewLine)
    $env:QT_QPA_PLATFORM_PLUGIN_PATH = Join-Path $InstallRoot ".pixi\envs\default\Library\plugins\platforms"
    . $rosSetup
    if ($env:ROS_DISTRO -ne "jazzy") {
        throw "Expected ROS_DISTRO=jazzy after sourcing $rosSetup, got '$env:ROS_DISTRO'."
    }
    $rosPrefix = Split-Path $rosSetup -Parent
    $rosFiles = @(
        (Join-Path $rosPrefix "bin\rcl.dll"),
        (Join-Path $rosPrefix "bin\rmw_fastrtps_cpp.dll"),
        (Join-Path $rosPrefix "share\ament_index\resource_index\packages\rclcpp")
    )
    foreach ($rosFile in $rosFiles) {
        if (-not (Test-Path $rosFile)) { throw "Required ROS 2 file was not found: $rosFile" }
    }
    Write-Host "Windows ROS 2 Jazzy is ready. ROS setup: $rosSetup"
}

if ($CheckOnly) {
    Test-Environment
    exit 0
}

New-Item -ItemType Directory -Path $InstallRoot -Force | Out-Null

if (-not (Test-Path $pixi)) {
    Download-File -Uri $pixiUrl -Destination $pixiArchive -MinimumBytes 10000000 -ExpectedSha256 $pixiSha256
    $pixiExtract = Join-Path $InstallRoot ".pixi-bootstrap"
    if (Test-Path $pixiExtract) { Remove-Item -LiteralPath $pixiExtract -Recurse -Force }
    Expand-Archive -LiteralPath $pixiArchive -DestinationPath $pixiExtract
    $extractedPixi = Get-ChildItem $pixiExtract -Filter pixi.exe -File -Recurse | Select-Object -First 1
    if (-not $extractedPixi) { throw "pixi.exe was not found in $pixiArchive" }
    Move-Item -LiteralPath $extractedPixi.FullName -Destination $pixi
    Remove-Item -LiteralPath $pixiExtract -Recurse -Force
}

Download-File -Uri $pixiManifestUrl -Destination $pixiManifest -MinimumBytes 1000
Write-Host "Installing the pinned ROS 2 dependency environment. This can take several minutes."
& $pixi install --manifest-path $pixiManifest
if ($LASTEXITCODE -ne 0) { throw "pixi install failed." }

if (-not (Find-RosSetup)) {
    Download-File -Uri $rosArchiveUrl -Destination $rosArchive -MinimumBytes 500000000 -ExpectedSha256 $rosSha256
    $rosDirectory = Join-Path $InstallRoot "ros2-windows"
    if (Test-Path $rosDirectory) {
        throw "$rosDirectory exists but local_setup.ps1 is missing. Move or remove that incomplete directory, then rerun."
    }
    Write-Host "Extracting ROS 2 Jazzy to $InstallRoot"
    Expand-Archive -LiteralPath $rosArchive -DestinationPath $InstallRoot
}

Test-Environment
Write-Host "Next, run: . .\unity\scripts\activate_ros2_jazzy_windows.ps1"
Write-Host "Then run: .\unity\scripts\setup.ps1"
