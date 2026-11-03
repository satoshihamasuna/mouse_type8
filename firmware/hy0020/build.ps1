param(
    [string]$SdkRoot = 'C:\ncs\v3.4.1',
    [string]$ToolchainRoot = 'C:\ncs\toolchains\4f5b6ad6dd',
    [switch]$Pristine
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$sdkPath = (Resolve-Path -LiteralPath $SdkRoot).Path
$toolchainPath = (Resolve-Path -LiteralPath $ToolchainRoot).Path
$environmentFile = Join-Path $toolchainPath 'environment.json'
$pythonExe = Join-Path $toolchainPath 'opt\bin\python.exe'
if (!(Test-Path -LiteralPath $pythonExe)) { throw "SDK Python not found: $pythonExe" }
$savedEnvironment = @{}
$savedLocation = Get-Location
try {
    # Use the bundled SDK environment, without modifying the user's settings.
    $bundle = Get-Content -LiteralPath $environmentFile -Raw | ConvertFrom-Json
    foreach ($entry in $bundle.env_vars) {
        $key = $entry.key
        $savedEnvironment[$key] = [Environment]::GetEnvironmentVariable($key, 'Process')
        if ($entry.type -eq 'relative_paths') {
            $value = ($entry.values | ForEach-Object { Join-Path $toolchainPath $_ }) -join ';'
            if ($entry.existing_value_treatment -eq 'prepend_to' -and $savedEnvironment[$key]) {
                $value += ';' + $savedEnvironment[$key]
            }
        } else {
            $value = $entry.value
        }
        [Environment]::SetEnvironmentVariable($key, $value, 'Process')
    }
    foreach ($key in @('ZEPHYR_BASE', 'ZEPHYR_CACHE_DIR')) {
        $savedEnvironment[$key] = [Environment]::GetEnvironmentVariable($key, 'Process')
    }
    $env:ZEPHYR_BASE = Join-Path $sdkPath 'zephyr'
    $env:ZEPHYR_CACHE_DIR = Join-Path $projectRoot 'build\hy0020\zephyr-cache'
    # Run west from the installed workspace so it can find the nRF modules.
    Set-Location $sdkPath
    $pristineMode = if ($Pristine) { 'always' } else { 'auto' }
    $buildArgs = @(
        '-m', 'west', 'build', '--no-sysbuild',
        '-b', 'hy0020_dip/nrf52832',
        '-d', (Join-Path $projectRoot 'build\hy0020\mouse_uart_ble'),
        '-p', $pristineMode,
        $PSScriptRoot,
        '--', "-DBOARD_ROOT=$($PSScriptRoot.Replace('\', '/'))",
        "-DUSER_CACHE_DIR=$($env:ZEPHYR_CACHE_DIR.Replace('\', '/'))",
        "-DPython3_EXECUTABLE=$($pythonExe.Replace('\', '/'))"
    )
    & $pythonExe @buildArgs
    if ($LASTEXITCODE -ne 0) { throw 'BLE build failed.' }
    Write-Host 'Built build/hy0020/mouse_uart_ble/zephyr/zephyr.hex'
} finally {
    Set-Location $savedLocation
    foreach ($key in $savedEnvironment.Keys) {
        [Environment]::SetEnvironmentVariable($key, $savedEnvironment[$key], 'Process')
    }
}
