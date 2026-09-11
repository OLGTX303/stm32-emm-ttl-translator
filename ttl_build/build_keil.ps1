param([string]$Keil = 'D:\Keil_v5\UV4\UV4.exe')
$ErrorActionPreference = 'Stop'
$project = Join-Path $PSScriptRoot 'PRJ\STM32_UART_CMD.uvprojx'
$out = Join-Path $PSScriptRoot 'build'
$log = Join-Path $out 'keil_build.log'
New-Item -ItemType Directory -Force $out | Out-Null
if (-not (Test-Path -LiteralPath $Keil)) { throw "Keil not found: $Keil" }
[System.IO.File]::WriteAllText($log, '')
$process = Start-Process -FilePath $Keil -WindowStyle Hidden -Wait -PassThru -ArgumentList @('-r', ('"' + $project + '"'), '-j0', '-o', ('"' + $log + '"'))
$report = Get-Content -LiteralPath $log -Raw
Write-Output $report
if ($process.ExitCode -gt 1 -or $report -notmatch '0 Error\(s\), 0 Warning\(s\)') {
    throw "Keil build did not finish cleanly; see $log"
}
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'PRJ\Objects\Template.hex') -Destination (Join-Path $out 'ttl_translator_keil.hex')
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'PRJ\Objects\Template.axf') -Destination (Join-Path $out 'ttl_translator_keil.axf')
Write-Output "Firmware: $out\ttl_translator_keil.hex"
