param([switch]$Real, [string]$IndexUrl = "https://pypi.org/simple")
$ErrorActionPreference = "Stop"
Set-Location -LiteralPath $PSScriptRoot
$bundledPython = Join-Path $PSScriptRoot "_local/runtime/python313/python.exe"
$venvPython = Join-Path $PSScriptRoot ".venv/Scripts/python.exe"

if (!(Test-Path -LiteralPath $venvPython)) {
    if (Test-Path -LiteralPath $bundledPython) {
        $basePython = $bundledPython
    } else {
        $basePython = (Get-Command python -ErrorAction Stop).Source
    }
    & $basePython -c "import sys; assert sys.version_info >= (3, 11), 'Python 3.11+ is required'"
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $basePython -m venv .venv
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

& $venvPython -m pip install --disable-pip-version-check --index-url $IndexUrl -r requirements.txt
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

if ($Real) {
    $tag = (& $venvPython -c "import sys; print(f'cp{sys.version_info.major}{sys.version_info.minor}')").Trim()
    $wheelName = "bpx_sdk_open-1.0.9-$tag-$tag-win_amd64.whl"
    $wheel = Join-Path $PSScriptRoot "_local/sdk/bpx_sdk_open/wheelhouse/$wheelName"
    if (!(Test-Path -LiteralPath $wheel)) { throw "Private SDK wheel is missing: $wheelName" }
    & $venvPython -m pip install --no-index --no-deps $wheel
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $venvPython -m dual_dog --real --check
} else {
    & $venvPython -m dual_dog --check
}
exit $LASTEXITCODE
