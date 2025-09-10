# Build script for host peripherals DLL
$ErrorActionPreference = "Stop"

# Create build directory if it doesn't exist
$buildDir = "$PSScriptRoot\..\firmware\build"
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

# Build the DLL
$gcc = Get-Command gcc -ErrorAction SilentlyContinue
if (-not $gcc) {
    Write-Error "gcc not found. Please install MinGW or ensure it's in your PATH"
    exit 1
}

Write-Host "Building libhostperipherals.dll..."
gcc -shared -o "$buildDir\libhostperipherals.dll" "$PSScriptRoot\..\firmware\Tests\host_peripherals.c" -lm

if ($LASTEXITCODE -eq 0) {
    Write-Host "Build successful!"
} else {
    Write-Error "Build failed with exit code $LASTEXITCODE"
    exit $LASTEXITCODE
}
