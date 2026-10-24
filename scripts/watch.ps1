# Watch mode (like nodemon): builds and starts the game, then on every save of
# code, shaders or CMake files it rebuilds and swaps in the new game.
#   - The running game stays open while compiling; if the build fails it keeps
#     running and the errors are shown here.
#   - config/*.json is NOT watched here: debug builds reload it live in-game.
# Uses watchexec (dev tool): winget install watchexec.watchexec
#
#   powershell -ExecutionPolicy Bypass -File scripts\watch.ps1 [-Config release]

param([ValidateSet('debug', 'release')][string]$Config = 'debug')
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

# Terminals opened before watchexec was installed have a stale PATH: refresh it
# from the registry, and fall back to winget's package folder.
if (-not (Get-Command watchexec -ErrorAction SilentlyContinue)) {
    $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
                [Environment]::GetEnvironmentVariable('Path', 'User')
}
if (-not (Get-Command watchexec -ErrorAction SilentlyContinue)) {
    $found = Get-ChildItem "$env:LOCALAPPDATA\Microsoft\WinGet\Packages\watchexec.watchexec*" -Recurse -Filter watchexec.exe -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($found) { $env:Path = $found.DirectoryName + ';' + $env:Path }
}
if (-not (Get-Command watchexec -ErrorAction SilentlyContinue)) {
    Write-Host 'watchexec not found. Install it with:  winget install watchexec.watchexec' -ForegroundColor Red
    Write-Host 'then open a new terminal.'
    exit 1
}

Write-Host "Watch mode ($Config): save a file to rebuild. Ctrl+C to stop watching." -ForegroundColor Cyan
# --on-busy-update queue: a save during a build triggers one more build after it.
# Runs once at startup (initial build + launch).
watchexec `
    --project-origin $root `
    --watch src --watch assets --watch cmake --watch CMakeLists.txt `
    --exts cpp,h,hpp,inl,mat,cmake,txt `
    --debounce 400ms `
    --on-busy-update queue `
    --shell none `
    -- powershell -NoProfile -ExecutionPolicy Bypass -File scripts\build.ps1 -Config $Config -NoTests -Detach
