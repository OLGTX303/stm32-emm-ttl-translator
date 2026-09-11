param(
    [string]$Python = 'C:\Users\TOM BROWN\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe',
    [string]$Compiler = 'gcc'
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$out = Join-Path $root 'build'
New-Item -ItemType Directory -Force $out | Out-Null
Push-Location -LiteralPath $root
try {
    & $Compiler -shared -std=c99 -O2 -Wall -Wextra -Werror -IAPP -IBSP APP/translator.c tests/host_shim.c -o build/translator_test.dll
    if ($LASTEXITCODE) { throw 'Host C build failed' }
    $report = & $Python tests/test_translator.py 2>&1
    $result = $LASTEXITCODE
    $report | Set-Content -Encoding utf8 build/test_results.log
    $report | Write-Output
    if ($result) { throw 'Translator tests failed' }
} finally {
    Pop-Location
}
