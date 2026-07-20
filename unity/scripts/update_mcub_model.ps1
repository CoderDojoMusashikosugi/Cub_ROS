param(
    [string]$WslDistribution = "Ubuntu",
    [string]$UnityEditor = "C:\Program Files\Unity\Hub\Editor\6000.5.3f1\Editor\Unity.exe",
    [switch]$SkipUnityValidation
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$wslRepo = (wsl.exe -d $WslDistribution -- wslpath -a ($repoRoot -replace '\\', '/')).Trim()
if ($LASTEXITCODE -ne 0) { throw "Failed to convert the repository path for WSL2." }

wsl.exe -d $WslDistribution -- bash -lc "cd '$wslRepo' && ./unity/scripts/update_mcub_model.sh"
if ($LASTEXITCODE -ne 0) { throw "xacro expansion failed." }

if (-not $SkipUnityValidation) {
    $unity = Start-Process -FilePath $UnityEditor -ArgumentList @(
        "-batchmode", "-quit", "-projectPath", "$repoRoot\unity\CubSim",
        "-executeMethod", "CubSim.Editor.CubSimProjectTools.ValidateProject",
        "-logFile", "$repoRoot\unity\update-mcub-model.log"
    ) -PassThru -Wait
    if ($unity.ExitCode -ne 0) { throw "Unity URDF validation failed. See unity/update-mcub-model.log." }
}
