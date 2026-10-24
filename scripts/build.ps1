# Configures, builds, tests and runs the game from a plain PowerShell window.
# Loads the MSVC x64 environment itself, so no Developer Prompt is needed.
#
#   powershell -ExecutionPolicy Bypass -File scripts\build.ps1                # debug: build + test + run
#   powershell -ExecutionPolicy Bypass -File scripts\build.ps1 -Config release
#   ... -NoRun / -NoTests / -ConfigureOnly / -Clean

param(
    [ValidateSet('debug', 'release')][string]$Config = 'debug',
    [switch]$NoRun,
    [switch]$NoTests,
    [switch]$ConfigureOnly,
    [switch]$Clean
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
if ($Clean -and (Test-Path $buildDir)) { Remove-Item $buildDir -Recurse -Force }

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
    $bin = Join-Path $buildDir 'bin'
    Push-Location $bin
    try { & .\CricketGame.exe } finally { Pop-Location }
}
