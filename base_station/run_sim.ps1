Param(
    [string]$venvPath = "$PSScriptRoot\.venv",
    [switch]$install
)

# First, build the host peripherals DLL
Write-Host "Building host peripherals DLL..."
& "$PSScriptRoot\build_peripherals.ps1"
if ($LASTEXITCODE -ne 0) {
    Write-Error "Failed to build host peripherals DLL"
    exit $LASTEXITCODE
}

# Set the DLL directory in PATH so Python can find it
$env:PATH = "$PSScriptRoot\..\firmware\build;$env:PATH"

if ($install) {
    python -m venv $venvPath
    & "$venvPath\Scripts\pip.exe" install -r "$PSScriptRoot\ui\..\requirements.txt"
}

#$activate = "$venvPath\Scripts\Activate.ps1"
#if (Test-Path $activate) { . $activate }

# Launch the GUI
& python "$PSScriptRoot\ui\sim_gui.py"
