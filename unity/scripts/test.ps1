param(
    [string]$UnityEditor = "C:\Program Files\Unity\Hub\Editor\6000.5.3f1\Editor\Unity.exe",
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
& (Join-Path $PSScriptRoot "update_mcub_model.ps1") -UnityEditor $UnityEditor -SkipUnityValidation
$unity = Start-Process -FilePath $UnityEditor -ArgumentList @(
    "-batchmode", "-quit", "-projectPath", "$repoRoot\unity\CubSim",
    "-executeMethod", "CubSim.Editor.CubSimProjectTools.ValidateProject",
    "-logFile", "$repoRoot\unity\test.log"
) -PassThru -Wait
if ($unity.ExitCode -ne 0) { throw "CubSim validation failed. See unity/test.log." }
