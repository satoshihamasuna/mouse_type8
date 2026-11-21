$ErrorActionPreference = 'Stop'
$adapterInstanceId = 'USB\VID_0BDA&PID_C761\6&3AF0F9CE&0&12'
$reportPath = Join-Path $PSScriptRoot '..\tools\logs\pixel_camera\turn1800_A_inr45_20261120_v1\mid_status_retry_assets\adapter_restart2017.json'
$startedAt = (Get-Date).ToUniversalTime().ToString('o')
try {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = [Security.Principal.WindowsPrincipal]::new($identity)
    if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
        throw 'Administrator approval is required for this device restart.'
    }
    if (Test-Path -LiteralPath $reportPath) { throw 'Existing report will not be overwritten.' }
    $before = Get-PnpDevice -InstanceId $adapterInstanceId
    if ($before.Class -ne 'Bluetooth' -or $before.FriendlyName -ne 'Generic Bluetooth Adapter') {
        throw 'Target device identity changed; restart cancelled.'
    }
    $commandOutput = (& pnputil.exe /restart-device $adapterInstanceId 2>&1 | Out-String)
    $commandExitCode = $LASTEXITCODE
    Start-Sleep -Milliseconds 1000
    $after = Get-PnpDevice -InstanceId $adapterInstanceId
    [ordered]@{
        started_at = $startedAt
        finished_at = (Get-Date).ToUniversalTime().ToString('o')
        administrator = $true
        instance_id = $adapterInstanceId
        name = $after.FriendlyName
        before_status = [string]$before.Status
        after_status = [string]$after.Status
        pnputil_exit_code = $commandExitCode
        pnputil_output = $commandOutput
        robot_power = 'Both OFF confirmed by operator'
        robot_commands_sent = 0
    } | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $reportPath -Encoding UTF8
} catch {
    Write-Error $_
    exit 1
}
