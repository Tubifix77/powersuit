<#
.SYNOPSIS
Flash a Docker-built node_bench role to a board from the Windows host.

.DESCRIPTION
Docker Desktop on Windows cannot pass a COM port into a container, so node_bench
is built in Docker (firmware/tools/build_bench.sh) and flashed from here with the
esptool in an ESP-IDF Python environment. See docs/bringup.md A.4d.

Before writing anything it checks that the build directory exists and holds the
role you asked for, because two boards with the same role look exactly like a
wiring fault (bringup.md A.4b).

The boards are shared with a sibling project that runs 24 h soaks on them.
Check docs/bench-log.md before flashing.

.EXAMPLE
powershell -File firmware/tools/flash_bench.ps1 -Role limb -Port COM4 -Monitor

.EXAMPLE
powershell -File firmware/tools/flash_bench.ps1 -Role orch -Port COM3 -DryRun
Prints the commands it would run and opens no port.
#>
param(
    [Parameter(Mandatory)] [ValidateSet('limb', 'orch')] [string] $Role,
    [Parameter(Mandatory)] [ValidatePattern('^COM\d+$')] [string] $Port,
    [switch] $Monitor,
    [switch] $DryRun,
    [int] $Baud = 460800
)
$ErrorActionPreference = 'Stop'

$repo  = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$build = Join-Path $repo "firmware\apps\node_bench\build_$Role"

# 1. A build exists, and it is the role asked for.
$argsFile = Join-Path $build 'flasher_args.json'
if (-not (Test-Path $argsFile)) {
    throw "no build in $build - run: bash firmware/tools/build_bench.sh"
}
$isOrch = Select-String -Path (Join-Path $build 'sdkconfig') -Pattern '^CONFIG_PS_BENCH_IS_ORCH=y$' -Quiet
if ([bool]$isOrch -ne ($Role -eq 'orch')) {
    throw "build_$Role\sdkconfig holds the WRONG role - rebuild with firmware/tools/build_bench.sh"
}

# 2. A Python that has esptool: PS_IDF_PYTHON, else the newest ESP-IDF environment.
$py = $env:PS_IDF_PYTHON
if (-not $py) {
    $py = Get-ChildItem "$env:USERPROFILE\.espressif\python_env\*\Scripts\python.exe" -ErrorAction SilentlyContinue |
          Sort-Object LastWriteTime -Descending | Select-Object -First 1 -ExpandProperty FullName
}
if (-not $py) {
    throw 'no ESP-IDF Python environment found - set PS_IDF_PYTHON to a python.exe that has esptool'
}

# 3. Translate the build's flasher_args.json into esptool 5 syntax. ESP-IDF 5.5
#    writes underscore options (--flash_mode); esptool 5 spells them with hyphens
#    and only warns on the old form, so convert rather than rely on the warning.
$fa    = Get-Content $argsFile -Raw | ConvertFrom-Json
$opts  = @($fa.write_flash_args | ForEach-Object { if ($_ -like '--*') { $_ -replace '_', '-' } else { $_ } })
$files = @($fa.flash_files.PSObject.Properties |
           Sort-Object { [Convert]::ToInt64($_.Name, 16) } |
           ForEach-Object { $_.Name; $_.Value })
$chip  = $fa.extra_esptool_args.chip

$flashCmd   = @('-m', 'esptool', '--chip', $chip, '-p', $Port, '-b', "$Baud", 'write-flash') + $opts + $files
$monitorCmd = @('-m', 'esp_idf_monitor', '-p', $Port, 'node_bench.elf')

Write-Host "node_bench role '$Role' -> $Port"
Write-Host "python : $py"
Write-Host "build  : $build"
Write-Host 'Shared boards: if a sibling-project soak is running, stop now (docs/bench-log.md).'

if ($DryRun) {
    Write-Host "`n[dry run - no port opened] in $build :"
    Write-Host "  & `"$py`" $($flashCmd -join ' ')"
    if ($Monitor) { Write-Host "  & `"$py`" $($monitorCmd -join ' ')" }
    return
}

Push-Location $build
try {
    & $py @flashCmd
    if ($LASTEXITCODE -ne 0) { throw "esptool failed (exit $LASTEXITCODE)" }
    if ($Monitor) {
        Write-Host 'monitor: Ctrl+] to exit'
        & $py @monitorCmd
    }
} finally {
    Pop-Location
}
