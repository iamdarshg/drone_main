# build_mingw.ps1 - helper to configure and build with MinGW on Windows
# Usage: .\build_mingw.ps1

$ErrorActionPreference = 'Stop'

Write-Host "Detecting gcc in PATH..."
$gcc = Get-Command gcc -ErrorAction SilentlyContinue
if (-not $gcc) {
    Write-Host "gcc not found in PATH, probing common MSYS2/MSYS locations..."
    $candidates = @(
        "C:\\msys64\\mingw64\\bin",
        "C:\\msys64\\usr\\bin",
        "C:\\Program Files\\mingw-w64\\mingw64\\bin",
        "C:\\MinGW\\bin"
    )
    $found = $false
    foreach ($p in $candidates) {
        if (Test-Path $p) {
            Write-Host "Adding $p to PATH"
            $env:PATH = "$p;$env:PATH"
            $found = $true
            break
        }
    }
    if (-not $found) {
        Write-Host "Could not find MinGW gcc. Please install MSYS2/MinGW and ensure gcc is in PATH." -ForegroundColor Red
        exit 1
    }
}

# Create build directory
$buildDir = Join-Path $PSScriptRoot 'build'
if (-not (Test-Path $buildDir)) { New-Item -ItemType Directory -Path $buildDir | Out-Null }

Push-Location $buildDir
try {
    Write-Host "Running cmake (MinGW Makefiles)..."
    & cmake .. -G "MinGW Makefiles"
    if ($LASTEXITCODE -ne 0) { throw "cmake configure failed" }

    # Try to pick mingw32-make or make
    $makeCmd = (Get-Command mingw32-make -ErrorAction SilentlyContinue) -or (Get-Command make -ErrorAction SilentlyContinue)
    if (-not $makeCmd) {
        Write-Host "Could not find mingw32-make or make in PATH. Install MSYS2/mingw and ensure make is available." -ForegroundColor Red
        exit 1
    }

    Write-Host "Building..."
    & $makeCmd.Path -j 4
    if ($LASTEXITCODE -ne 0) { throw "make failed" }
    Write-Host "Build finished successfully"
} finally {
    Pop-Location
}
