param([switch]$Real, [switch]$Check)
$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot
$python = Join-Path $PSScriptRoot ".venv/Scripts/python.exe"
if (!(Test-Path -LiteralPath $python)) { throw "Run setup.ps1 first (add -Real for hardware)." }

$env:PYTHONUTF8 = "1"
$arguments = @("-m", "dual_dog")
if ($Real) { $arguments += "--real" }
if ($Check) { $arguments += "--check" }
& $python @arguments
exit $LASTEXITCODE
