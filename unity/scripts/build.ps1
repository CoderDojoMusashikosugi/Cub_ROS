param(
    [ValidateSet("Windows")][string]$Target = "Windows",
    [string]$UnityEditor = "C:\Program Files\Unity\Hub\Editor\6000.4.2f1\Editor\Unity.exe"
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
& (Join-Path $PSScriptRoot "update_mcub_model.ps1") -UnityEditor $UnityEditor -SkipUnityValidation
$unity = Start-Process -FilePath $UnityEditor -ArgumentList @(
    "-batchmode", "-quit", "-projectPath", "$repoRoot\unity\CubSim",
    "-executeMethod", "CubSim.Editor.CubSimProjectTools.BuildWindows",
    "-logFile", "$repoRoot\unity\build.log"
) -PassThru -Wait
if ($unity.ExitCode -ne 0) { throw "Unity Player build failed. See unity/build.log." }
