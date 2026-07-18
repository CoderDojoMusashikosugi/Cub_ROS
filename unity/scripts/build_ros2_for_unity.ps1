param(
    [switch]$Clean,
    [string]$WorkRoot = "C:\r"
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$sourceRoot = Join-Path $repoRoot "unity\ros2-for-unity"
$buildRoot = Join-Path $WorkRoot "b"
$installRoot = Join-Path $WorkRoot "i"
$logRoot = Join-Path $WorkRoot "l"
Push-Location $sourceRoot
try {
    if ($env:ROS_DISTRO -ne "jazzy") { throw "ROS 2 Jazzy must be sourced before this script is run." }
    if ($Clean) {
        foreach ($path in @($buildRoot, $installRoot, $logRoot, (Join-Path $sourceRoot "install\asset"))) {
            if (Test-Path $path) { Remove-Item -Recurse -Force -LiteralPath $path }
        }
    }

    & python "src\scripts\metadata_generator.py" --standalone
    if ($LASTEXITCODE -ne 0) { throw "ros2-for-unity metadata generation failed." }

    New-Item -ItemType Directory -Force $WorkRoot | Out-Null
    & colcon --log-base $logRoot build --merge-install --build-base $buildRoot --install-base $installRoot `
        --event-handlers console_direct+ `
        --cmake-args "-G" "Ninja" "-DSTANDALONE_BUILD:int=1" `
        "-DCMAKE_BUILD_TYPE=Release" "-DBUILD_TESTING:int=0" `
        "-DCMAKE_OBJECT_PATH_MAX=260" "--no-warn-unused-cli"
    if ($LASTEXITCODE -ne 0) { throw "ros2cs colcon build failed." }

    $assetRoot = Join-Path $sourceRoot "install\asset"
    $asset = Join-Path $assetRoot "Ros2ForUnity"
    New-Item -ItemType Directory -Force $assetRoot | Out-Null
    Copy-Item -Path (Join-Path $sourceRoot "src\Ros2ForUnity") -Destination $assetRoot -Recurse -Force
    $pluginRoot = Join-Path $asset "Plugins"
    $nativeRoot = Join-Path $pluginRoot "Windows\x86_64"
    New-Item -ItemType Directory -Force $nativeRoot | Out-Null
    Get-ChildItem (Join-Path $installRoot "lib\dotnet") -File -Recurse -Exclude "*.pdb" |
        Copy-Item -Destination $pluginRoot -Force
    Get-ChildItem (Join-Path $installRoot "bin") -File -Recurse -Exclude "*_py.dll", "*_python.dll" |
        Copy-Item -Destination $nativeRoot -Force
    foreach ($directory in @("standalone", "resources")) {
        $path = Join-Path $installRoot $directory
        if (Test-Path $path) {
            Get-ChildItem $path -File -Filter "*.dll" | Copy-Item -Destination $nativeRoot -Force
        }
    }

    # The official Jazzy Windows archive is installed in a pixi environment.
    # A few native ROS libraries link to pixi-provided runtime DLLs instead of
    # carrying those DLLs in the ROS archive itself.  They must be present next
    # to rcl.dll in a standalone Unity Player.
    $pixiRuntime = if ($env:CONDA_PREFIX) {
        Join-Path $env:CONDA_PREFIX "Library\bin"
    } else {
        $null
    }
    $pixiDependencies = @(
        "yaml.dll", "spdlog.dll", "fmt.dll", "console_bridge.dll",
        "libcrypto-3-x64.dll", "libssl-3-x64.dll"
    )
    foreach ($dependency in $pixiDependencies) {
        $source = if ($pixiRuntime) { Join-Path $pixiRuntime $dependency } else { $null }
        if (-not $source -or -not (Test-Path $source)) {
            throw "$dependency was not found in the active pixi environment. Run this script from the ROS 2 Jazzy pixi shell."
        }
        Copy-Item $source $nativeRoot -Force
    }

    # Jazzy resolves the selected RMW implementation through the ament index at
    # runtime.  Keep the small rmw_typesupport index beside the standalone DLLs.
    $amentResource = $null
    foreach ($prefix in ($env:AMENT_PREFIX_PATH -split ';')) {
        if (-not $prefix) { continue }
        $candidate = Join-Path $prefix "share\ament_index\resource_index\rmw_typesupport"
        if (Test-Path $candidate) {
            $amentResource = $candidate
            break
        }
    }
    if (-not $amentResource) {
        throw "The rmw_typesupport ament index was not found. Source ROS 2 Jazzy before running this script."
    }
    $standaloneAmentResource = Join-Path $nativeRoot "share\ament_index\resource_index\rmw_typesupport"
    New-Item -ItemType Directory -Force $standaloneAmentResource | Out-Null
    Get-ChildItem $amentResource -File | Copy-Item -Destination $standaloneAmentResource -Force

    $metadata = Join-Path $sourceRoot "src\Ros2ForUnity\metadata_ros2cs.xml"
    Copy-Item $metadata (Join-Path $asset "Plugins") -Force
    Copy-Item $metadata (Join-Path $asset "Plugins\Windows\x86_64") -Force
    if (-not (Test-Path (Join-Path $asset "Plugins\ros2cs_core.dll"))) {
        throw "ros2cs_core.dll was not deployed."
    }
} finally {
    Pop-Location
}
