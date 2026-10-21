[CmdletBinding()]
param(
    [string]$GodotPath = 'C:\Users\tomok\workspace\Godot_v4.7.2-stable_win64.exe',
    [ValidateRange(0, 3)][int]$Clients = 3,
    [ValidateRange(1024, 65535)][int]$Port = 27140,
    [ValidateRange(0, 3600)][int]$RunSeconds = 0,
    [switch]$SkipImport,
    [switch]$PassThru
)

$ErrorActionPreference = 'Stop'
$projectPath = $PSScriptRoot
if (Test-Path -LiteralPath $GodotPath -PathType Container) {
    $GodotPath = Join-Path $GodotPath 'Godot_v4.7.2-stable_win64.exe'
}
if (-not (Test-Path -LiteralPath $GodotPath -PathType Leaf)) {
    throw "Godot executable was not found: $GodotPath"
}
$logDirectory = Join-Path $projectPath '.godot\local_sessions'
New-Item -ItemType Directory -Force -Path $logDirectory | Out-Null

if (-not $SkipImport) {
    $importArguments = @('--headless', '--editor', '--path', ('"' + $projectPath + '"'),
        '--import', '--quit', '--log-file', ('"' + (Join-Path $logDirectory 'import.log') + '"'))
    $import = Start-Process -FilePath $GodotPath -ArgumentList $importArguments -Wait -PassThru
    if ($import.ExitCode -ne 0) {
        throw "Godot import failed. See $logDirectory\import.log"
    }
}

Add-Type -AssemblyName System.Windows.Forms
$area = [System.Windows.Forms.Screen]::PrimaryScreen.WorkingArea
$width = [Math]::Max(640, [int]([Math]::Floor($area.Width / 2) - 12))
$height = [Math]::Max(420, [int]([Math]::Floor($area.Height / 2) - 38))
$kinds = @('robot', 'pedestrian', 'robot', 'cone')
$names = @('Host', 'Walker', 'Robot', 'Cone')
$processes = @()
try {
    for ($index = 0; $index -le $Clients; $index++) {
        $x = $area.Left + ($index % 2) * [int]($area.Width / 2)
        $y = $area.Top + [Math]::Floor($index / 2) * [int]($area.Height / 2) + 28
        $arguments = @('--path', ('"' + $projectPath + '"'), '--resolution', "${width}x${height}",
            '--position', "$x,$y", '--audio-driver', 'Dummy', '--log-file',
            ('"' + (Join-Path $logDirectory ("window-$index.log")) + '"'), '--')
        if ($index -eq 0) { $arguments += '--host' }
        else { $arguments += @('--join', '127.0.0.1') }
        $arguments += @('--port', "$Port", '--actor', $kinds[$index], '--name', $names[$index])
        if ($RunSeconds -gt 0) { $arguments += @('--run-seconds', "$RunSeconds") }
        $process = Start-Process -FilePath $GodotPath -ArgumentList $arguments -PassThru
        $processes += $process
        Write-Host ("{0}: PID {1}, {2}" -f $names[$index], $process.Id, $kinds[$index])
        # Give the host time to create its ENet socket before the first client.
        if ($index -eq 0) { Start-Sleep -Milliseconds 1200 }
        else { Start-Sleep -Milliseconds 200 }
    }
}
catch {
    # Only stop processes that this invocation started.
    $processes | Where-Object { -not $_.HasExited } | Stop-Process -ErrorAction SilentlyContinue
    throw
}
Write-Host "Started one host and $Clients client(s). Close each window or use Leave to end a session."
if ($PassThru) { $processes }
