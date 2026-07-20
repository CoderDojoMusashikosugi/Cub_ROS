param(
    [string]$InstallRoot = "C:\pixi_ws"
)

$previousErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = "Stop"
$InstallRoot = [IO.Path]::GetFullPath($InstallRoot).TrimEnd('\')
$pixi = Join-Path $InstallRoot "pixi.exe"
$pixiManifest = Join-Path $InstallRoot "pixi.toml"
$pixiPrefix = Join-Path $InstallRoot ".pixi\envs\default"
$rosSetupCandidates = @(
    (Join-Path $InstallRoot "ros2-windows\local_setup.ps1"),
    (Join-Path $InstallRoot "ros2-windows\ros2-windows\local_setup.ps1")
)
$rosSetup = $rosSetupCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1

$samePath = {
    param([string]$Left, [string]$Right)

    if ([string]::IsNullOrWhiteSpace($Left) -or [string]::IsNullOrWhiteSpace($Right)) {
        return $false
    }
    try {
        $normalizedLeft = [IO.Path]::GetFullPath($Left).TrimEnd('\', '/')
        $normalizedRight = [IO.Path]::GetFullPath($Right).TrimEnd('\', '/')
        return [string]::Equals($normalizedLeft, $normalizedRight, [StringComparison]::OrdinalIgnoreCase)
    } catch {
        return $false
    }
}

$pathListContains = {
    param([string]$PathList, [string]$ExpectedPath)

    if ([string]::IsNullOrWhiteSpace($PathList)) { return $false }
    foreach ($entry in $PathList.Split(';', [StringSplitOptions]::RemoveEmptyEntries)) {
        if (& $samePath $entry $ExpectedPath) { return $true }
    }
    return $false
}

$setProcessEnvironmentVariable = {
    param([string]$Name, [AllowNull()][string]$Value)

    # Windows variable names are case-insensitive, but a process environment
    # block can still contain case-only duplicates. Remove every spelling first.
    $existingNames = @([Environment]::GetEnvironmentVariables([EnvironmentVariableTarget]::Process).Keys |
        Where-Object { [string]::Equals([string]$_, $Name, [StringComparison]::OrdinalIgnoreCase) })
    foreach ($existingName in $existingNames) {
        [Environment]::SetEnvironmentVariable([string]$existingName, $null, [EnvironmentVariableTarget]::Process)
    }
    if ($null -ne $Value) {
        [Environment]::SetEnvironmentVariable($Name, $Value, [EnvironmentVariableTarget]::Process)
    }
}

$environmentSnapshot = [Collections.Generic.Dictionary[string, string]]::new([StringComparer]::OrdinalIgnoreCase)
foreach ($entry in [Environment]::GetEnvironmentVariables([EnvironmentVariableTarget]::Process).GetEnumerator()) {
    $environmentSnapshot[[string]$entry.Key] = [string]$entry.Value
}
$originalPrompt = Get-Item -Path Function:\prompt -ErrorAction SilentlyContinue
$originalPromptScriptBlock = if ($originalPrompt) { $originalPrompt.ScriptBlock } else { $null }
$originalOldPromptVariable = Get-Variable -Name old_prompt -Scope Local -ErrorAction SilentlyContinue
$originalOldPromptVariableWasDefined = $null -ne $originalOldPromptVariable
$originalOldPromptVariableValue = if ($originalOldPromptVariableWasDefined) { $originalOldPromptVariable.Value } else { $null }
$originalOutputEncoding = $OutputEncoding
$originalConsoleInputEncoding = [Console]::InputEncoding
$originalConsoleOutputEncoding = [Console]::OutputEncoding

try {
    foreach ($required in @($pixi, $pixiManifest, $rosSetup)) {
        if (-not $required -or -not (Test-Path $required)) {
            throw "Windows ROS 2 Jazzy is not installed under $InstallRoot. Run unity\scripts\install_ros2_jazzy_windows.ps1 first."
        }
    }

    $pixiAlreadyActive =
        -not [string]::IsNullOrWhiteSpace($env:PIXI_IN_SHELL) -and
        (& $samePath $env:CONDA_PREFIX $pixiPrefix) -and
        (& $samePath $env:PIXI_PROJECT_ROOT $InstallRoot) -and
        ([string]::IsNullOrWhiteSpace($env:PIXI_PROJECT_MANIFEST) -or (& $samePath $env:PIXI_PROJECT_MANIFEST $pixiManifest))
    $anyPixiEnvironmentActive =
        -not [string]::IsNullOrWhiteSpace($env:PIXI_IN_SHELL) -or
        -not [string]::IsNullOrWhiteSpace($env:PIXI_PROJECT_ROOT) -or
        -not [string]::IsNullOrWhiteSpace($env:PIXI_PROJECT_MANIFEST)

    if ($anyPixiEnvironmentActive -and -not $pixiAlreadyActive) {
        throw "A different or incomplete Pixi environment is already active. Start a new PowerShell, then source this script again."
    }

    if (-not $pixiAlreadyActive) {
        # Pixi warns before emitting the hook if an inherited SSL_CERT_DIR is
        # empty. The environment supplies a valid SSL_CERT_FILE, so omit only
        # an unusable certificate directory.
        if (-not [string]::IsNullOrWhiteSpace($env:SSL_CERT_DIR)) {
            $certificates = @(Get-ChildItem -LiteralPath $env:SSL_CERT_DIR -File -ErrorAction SilentlyContinue |
                Where-Object { $_.Extension -in @('.crt', '.pem') })
            if ($certificates.Count -eq 0) {
                & $setProcessEnvironmentVariable "SSL_CERT_DIR" $null
            }
        }

        $hook = & $pixi shell-hook --manifest-path $pixiManifest --shell powershell
        if ($LASTEXITCODE -ne 0) { throw "pixi environment activation failed." }
        Invoke-Expression ($hook -join [Environment]::NewLine)
    }

    $env:QT_QPA_PLATFORM_PLUGIN_PATH = Join-Path $pixiPrefix "Library\plugins\platforms"
    if (-not [string]::IsNullOrWhiteSpace($env:SSL_CERT_DIR)) {
        $certificates = @(Get-ChildItem -LiteralPath $env:SSL_CERT_DIR -File -ErrorAction SilentlyContinue |
            Where-Object { $_.Extension -in @('.crt', '.pem') })
        if ($certificates.Count -eq 0 -and (Test-Path $env:SSL_CERT_FILE -PathType Leaf)) {
            & $setProcessEnvironmentVariable "SSL_CERT_DIR" $null
        }
    }

    $rosPrefix = Split-Path $rosSetup -Parent
    $rosAlreadyActive =
        $env:ROS_DISTRO -eq "jazzy" -and
        (& $pathListContains $env:AMENT_PREFIX_PATH $rosPrefix)
    if (-not [string]::IsNullOrWhiteSpace($env:ROS_DISTRO) -and -not $rosAlreadyActive) {
        throw "A different ROS environment is already active. Start a new PowerShell, then source this script again."
    }
    if (-not $rosAlreadyActive) {
        . $rosSetup
    }

    if ($env:ROS_DISTRO -ne "jazzy") {
        throw "Expected ROS_DISTRO=jazzy, got '$env:ROS_DISTRO'."
    }

    $clCommand = Get-Command cl.exe -ErrorAction SilentlyContinue
    $visualStudioAlreadyActive =
        -not [string]::IsNullOrWhiteSpace($env:VSCMD_VER) -and
        $null -ne $clCommand

    if (-not $visualStudioAlreadyActive) {
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

        # Import the x64 MSVC environment into this PowerShell. Launch-VsDevShell.ps1
        # can fail on localized Visual Studio metadata, so use the batch entry point.
        $devCommand = '"' + $vsDevCmd + '" -no_logo -arch=x64 -host_arch=x64 && set'
        $devEnvironment = & $env:ComSpec /d /s /c $devCommand
        if ($LASTEXITCODE -ne 0) { throw "Visual Studio x64 developer environment activation failed." }

        $devVariables = [Collections.Generic.Dictionary[string, string]]::new([StringComparer]::OrdinalIgnoreCase)
        $developerPaths = [Collections.Generic.List[string]]::new()
        foreach ($line in $devEnvironment) {
            if ($line -notmatch '^([^=]+)=(.*)$') { continue }
            $name = $Matches[1]
            $value = $Matches[2]
            if ($name -ieq "Path") {
                $developerPaths.Add($value)
            } else {
                $devVariables[$name] = $value
            }
        }

        $developerPath = $developerPaths |
            Where-Object { $_ -match '[\\/]VC[\\/]Tools[\\/]MSVC[\\/]' } |
            Select-Object -First 1
        if (-not $developerPath) {
            $developerPath = $developerPaths | Select-Object -First 1
        }
        if (-not $developerPath) {
            throw "Visual Studio developer PATH was not returned by VsDevCmd.bat."
        }

        foreach ($entry in $devVariables.GetEnumerator()) {
            & $setProcessEnvironmentVariable $entry.Key $entry.Value
        }
        & $setProcessEnvironmentVariable "Path" $developerPath
    }

    $clCommand = Get-Command cl.exe -ErrorAction SilentlyContinue
    if (-not $clCommand -or $clCommand.Source -notmatch '[\\/]VC[\\/]Tools[\\/]MSVC[\\/]') {
        throw "MSVC cl.exe was not found under Visual Studio VC\Tools\MSVC. Add the Desktop development with C++ workload and a Windows SDK in Visual Studio Installer."
    }
    $expectedPixiBin = Join-Path $pixiPrefix "Library\bin"
    foreach ($toolName in @("ninja.exe", "cmake.exe")) {
        $tool = Get-Command $toolName -ErrorAction SilentlyContinue
        if (-not $tool -or -not (& $samePath (Split-Path $tool.Source -Parent) $expectedPixiBin)) {
            throw "$toolName was not found in $expectedPixiBin. Rerun install_ros2_jazzy_windows.ps1 without -CheckOnly."
        }
    }

    Write-Host "ROS 2 $env:ROS_DISTRO activated from $rosSetup"
} catch {
    $activationError = $_
    try {
        $currentEnvironment = [Environment]::GetEnvironmentVariables([EnvironmentVariableTarget]::Process)
        foreach ($name in @($currentEnvironment.Keys)) {
            if (-not $environmentSnapshot.ContainsKey([string]$name)) {
                & $setProcessEnvironmentVariable ([string]$name) $null
            }
        }
        foreach ($entry in $environmentSnapshot.GetEnumerator()) {
            & $setProcessEnvironmentVariable $entry.Key $entry.Value
        }

        if ($originalPromptScriptBlock) {
            Set-Item -Path Function:\prompt -Value $originalPromptScriptBlock
        } else {
            Remove-Item -Path Function:\prompt -ErrorAction SilentlyContinue
        }
        if ($originalOldPromptVariableWasDefined) {
            Set-Variable -Name old_prompt -Scope Local -Value $originalOldPromptVariableValue
        } else {
            Remove-Variable -Name old_prompt -Scope Local -ErrorAction SilentlyContinue
        }
        $OutputEncoding = $originalOutputEncoding
        [Console]::InputEncoding = $originalConsoleInputEncoding
        [Console]::OutputEncoding = $originalConsoleOutputEncoding
    } catch {
        Write-Warning "Activation failed and the original PowerShell environment could not be fully restored: $($_.Exception.Message)"
    }
    throw $activationError
} finally {
    $ErrorActionPreference = $previousErrorActionPreference
}
