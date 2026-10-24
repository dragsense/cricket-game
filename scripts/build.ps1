# Configures, builds, tests and runs the game from a plain PowerShell window.
# Loads the MSVC x64 environment itself, so no Developer Prompt is needed.
#
#   powershell -ExecutionPolicy Bypass -File scripts\build.ps1                # debug: build + test + run
#   powershell -ExecutionPolicy Bypass -File scripts\build.ps1 -Config release
#   ... -NoRun / -NoTests / -ConfigureOnly / -Clean / -Detach
#
# A running game keeps running while it rebuilds: its locked exe is renamed
# aside (Windows allows that) so the linker can write the new one. The old game
# is only closed when the new one starts, so a failed build leaves it running.

param(
    [ValidateSet('debug', 'release')][string]$Config = 'debug',
    [switch]$NoRun,
    [switch]$NoTests,
    [switch]$ConfigureOnly,
    [switch]$Clean,
    [switch]$Detach   # start the game without waiting for it (used by watch mode)
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

function Import-MsvcEnv {
    if (Get-Command cl.exe -ErrorAction SilentlyContinue) { return }
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    $vs = & $vswhere -latest -products * -version '[17.0,18.0)' `
        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if (-not $vs) { throw 'Visual Studio 2022 C++ tools not found. Run scripts\setup_windows.ps1.' }
    $vcvars = Join-Path $vs.Trim() 'VC\Auxiliary\Build\vcvars64.bat'
    # Run vcvars in cmd and import the resulting environment into this process.
    cmd /c "`"$vcvars`" >nul && set" | ForEach-Object {
        if ($_ -match '^([^=]+)=(.*)$') { Set-Item -Path "env:$($Matches[1])" -Value $Matches[2] }
    }
}

Import-MsvcEnv
$buildDir = Join-Path $root "build\$Config"
$bin = Join-Path $buildDir 'bin'
$exe = Join-Path $bin 'CricketGame.exe'

# Games started from this build folder (including renamed-aside copies).
function Get-GameProcesses {
    Get-Process CricketGame* -ErrorAction SilentlyContinue |
        Where-Object { $_.Path -and $_.Path.StartsWith($bin, [StringComparison]::OrdinalIgnoreCase) }
}
function Stop-Game {
    Get-GameProcesses | ForEach-Object {
        Write-Host "Closing previous game (pid $($_.Id))..."
        $_ | Stop-Process -Force
        $_.WaitForExit()
    }
}

if ($Clean) {
    Stop-Game
    if (Test-Path $buildDir) { Remove-Item $buildDir -Recurse -Force }
} else {
    # Delete leftovers from earlier runs (skips any still in use), then move the
    # running exe aside so the linker can write a new one.
    Get-ChildItem $bin -Filter 'CricketGame.*.old.exe' -ErrorAction SilentlyContinue |
        ForEach-Object { Remove-Item $_.FullName -Force -ErrorAction SilentlyContinue }
    if ((Test-Path $exe) -and (Get-GameProcesses)) {
        $aside = "CricketGame.$([DateTime]::Now.ToString('HHmmssfff')).old.exe"
        Rename-Item $exe $aside
        Write-Host "Game is running: keeping it open while rebuilding."
    }
}

cmake --preset $Config
if ($LASTEXITCODE -ne 0) { throw 'CMake configure failed' }
if ($ConfigureOnly) { exit 0 }

cmake --build --preset $Config
if ($LASTEXITCODE -ne 0) { throw 'Build failed' }

if (-not $NoTests) {
    ctest --preset $Config
    if ($LASTEXITCODE -ne 0) { throw 'Tests failed' }
}

if (-not $NoRun) {
    Stop-Game  # the new build is ready: swap it in
    if ($Detach) {
        Start-Process -FilePath $exe -WorkingDirectory $bin | Out-Null
        Write-Host "Game started ($Config). Watching for changes..."
    } else {
        Push-Location $bin
        try { & .\CricketGame.exe } finally { Pop-Location }
    }
}
