param()

$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$docsInstallerDir = Join-Path $projectRoot "docs\installer"
$pio = Get-Command pio -ErrorAction SilentlyContinue

if (-not $pio) {
  $fallbackPio = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\platformio.exe"
  if (Test-Path $fallbackPio) {
    $pio = $fallbackPio
  } else {
    throw "PlatformIO was not found. Install PlatformIO or add pio to PATH."
  }
} else {
  $pio = $pio.Source
}

$python = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"
$esptool = Join-Path $env:USERPROFILE ".platformio\packages\tool-esptoolpy\esptool.py"
$bootApp = Join-Path $env:USERPROFILE ".platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin"

Push-Location $projectRoot
try {
  & $pio run -e esp32dev

  New-Item -ItemType Directory -Force -Path $docsInstallerDir | Out-Null

  $bootloader = Join-Path $projectRoot ".pio\build\esp32dev\bootloader.bin"
  $partitions = Join-Path $projectRoot ".pio\build\esp32dev\partitions.bin"
  $firmware = Join-Path $projectRoot ".pio\build\esp32dev\firmware.bin"
  $factoryOut = Join-Path $docsInstallerDir "water-meter-esp32-factory.bin"
  $otaOut = Join-Path $docsInstallerDir "water-meter-esp32-ota.bin"

  & $python $esptool --chip esp32 merge_bin `
    -o $factoryOut `
    --flash_mode dio `
    --flash_freq 40m `
    --flash_size 4MB `
    0x1000 $bootloader `
    0x8000 $partitions `
    0xe000 $bootApp `
    0x10000 $firmware

  Copy-Item $firmware $otaOut -Force

  Write-Host "Browser installer artifacts updated:"
  Write-Host "  $factoryOut"
  Write-Host "  $otaOut"
} finally {
  Pop-Location
}
