# Checks the Windows toolchain, downloads the pinned Filament release into
# third_party/filament and configures the debug preset.
#
#   powershell -ExecutionPolicy Bypass -File scripts\setup_windows.ps1 [-Force]

param([switch]$Force)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot

function Find-VsInstall {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path $vswhere)) { return $null }
    $path = & $vswhere -latest -products * -version '[17.0,18.0)' `
        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if ($path) { return $path.Trim() }
    return $null
}

# ---- 1. Tool checks ---------------------------------------------------------
$missing = @()
$vs = Find-VsInstall
if ($vs) { Write-Host "[ok] Visual Studio 2022 C++ tools: $vs" }
else {
    $missing += 'Visual Studio 2022 Build Tools (C++ workload):' +
        "`n      winget install Microsoft.VisualStudio.2022.BuildTools --override `"--quiet --wait --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended`"" +
        "`n      or https://visualstudio.microsoft.com/visual-cpp-build-tools/ (select 'Desktop development with C++')"
}

$tools = @(
    @{ Name = 'cmake'; Install = 'winget install Kitware.CMake   (or https://cmake.org/download/, need 3.28+)' },
    @{ Name = 'ninja'; Install = 'winget install Ninja-build.Ninja' },
    @{ Name = 'git';   Install = 'winget install Git.Git' }
)
foreach ($t in $tools) {
    $cmd = Get-Command $t.Name -ErrorAction SilentlyContinue
    if ($cmd) { Write-Host "[ok] $($t.Name): $($cmd.Source)" }
    else { $missing += "$($t.Name): $($t.Install)" }
}

if (Get-Command cmake -ErrorAction SilentlyContinue) {
    $ver = [version]((cmake --version | Select-Object -First 1) -replace '[^\d\.]', '')
    if ($ver -lt [version]'3.28') { $missing += "cmake $ver is too old, need 3.28+: winget upgrade Kitware.CMake" }
}

if ($missing.Count -gt 0) {
    Write-Host "`nMissing tools:" -ForegroundColor Red
    $missing | ForEach-Object { Write-Host "  - $_" }
    Write-Host "`nInstall them, open a NEW terminal so PATH refreshes, then re-run this script."
    exit 1
}

# ---- 2. Filament ------------------------------------------------------------
$versions = Get-Content (Join-Path $root 'cmake\dependencies.cmake') -Raw
if ($versions -notmatch 'set\(FILAMENT_VERSION\s+"([^"]+)"\)') { throw 'FILAMENT_VERSION not found in cmake/dependencies.cmake' }
$filVer = $Matches[1]
$filDir = Join-Path $root 'third_party\filament'
$stamp = Join-Path $filDir 'VERSION.txt'

if ($Force -or -not (Test-Path $stamp) -or ((Get-Content $stamp -Raw).Trim() -ne $filVer)) {
    $url = "https://github.com/google/filament/releases/download/$filVer/filament-$filVer-windows.tgz"
    $tmp = Join-Path $root 'third_party\_download'
    New-Item -ItemType Directory -Force $tmp | Out-Null
    $tgz = Join-Path $tmp "filament-$filVer.tgz"
    Get-ChildItem $tmp -Directory | Remove-Item -Recurse -Force  # stale partial extracts
    Write-Host "Downloading Filament $filVer ..."
    # curl.exe ships with Windows 10+ and is far faster than Invoke-WebRequest
    # for this ~850 MB archive; -C - resumes an interrupted download.
    curl.exe -L -C - --retry 20 --retry-all-errors --retry-delay 3 --fail -o $tgz $url
    if ($LASTEXITCODE -ne 0) { throw "Download failed: $url" }
    Write-Host 'Extracting ...'
    $extracted = Join-Path $tmp 'extract'
    New-Item -ItemType Directory -Force $extracted | Out-Null
    tar -xzf $tgz -C $extracted
    if ($LASTEXITCODE -ne 0) { throw 'tar failed to extract Filament' }
    # Some releases wrap everything in a top-level 'filament' folder.
    if (Test-Path (Join-Path $extracted 'filament\include')) { $extracted = Join-Path $extracted 'filament' }
    if (Test-Path $filDir) { Remove-Item $filDir -Recurse -Force }
    Move-Item $extracted $filDir
    Remove-Item $tmp -Recurse -Force -ErrorAction SilentlyContinue
    Set-Content -Path $stamp -Value $filVer -Encoding ascii
    Write-Host "[ok] Filament $filVer -> $filDir"
} else {
    Write-Host "[ok] Filament $filVer already present"
}

# ---- 3. Configure -----------------------------------------------------------
if ($env:SKIP_CONFIGURE) { exit 0 }
& (Join-Path $PSScriptRoot 'build.ps1') -ConfigureOnly
