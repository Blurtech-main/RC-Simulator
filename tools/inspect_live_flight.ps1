param([ValidateRange(1,120)][int]$Seconds = 15)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$snapshotPath = Join-Path $projectRoot 'reports\troubleshooting\live-flight-snapshot.json'
$pointerPath = Join-Path $projectRoot 'reports\troubleshooting\current.ptr'
if (-not (Test-Path -LiteralPath $snapshotPath)) { throw 'No live flight snapshot exists. Start RCTrainer.exe first.' }
if (-not (Test-Path -LiteralPath $pointerPath)) { throw 'No troubleshooting session pointer exists.' }
$snapshot = Get-Content -LiteralPath $snapshotPath -Raw | ConvertFrom-Json
$archiveName = (Get-Content -LiteralPath $pointerPath -Raw).Trim()
$archivePath = if ([IO.Path]::IsPathRooted($archiveName)) { $archiveName } else { Join-Path $projectRoot $archiveName }
$events = @(Get-Content -LiteralPath $archivePath | ForEach-Object { try { $_ | ConvertFrom-Json } catch {} })
$samples = @($events | Where-Object kind -eq 'sample' | Select-Object -Last ($Seconds * 10))
$issues = @($events | Where-Object kind -eq 'user_issue' | Select-Object -Last 8)
if (-not $samples.Count) { throw 'The live session has not produced telemetry samples yet.' }
$latest = $samples[-1]
$summary = [ordered]@{
  schema = 1
  requested_window_s = $Seconds
  session_id = $snapshot.session_id
  archive = $archivePath
  samples = $samples.Count
  latest = $snapshot
  window = [ordered]@{
    altitude_min_m = [math]::Round(($samples.state.y_m | Measure-Object -Minimum).Minimum, 3)
    altitude_max_m = [math]::Round(($samples.state.y_m | Measure-Object -Maximum).Maximum, 3)
    vertical_speed_min_mps = [math]::Round(($samples.state.vy_mps | Measure-Object -Minimum).Minimum, 3)
    vertical_speed_max_mps = [math]::Round(($samples.state.vy_mps | Measure-Object -Maximum).Maximum, 3)
    max_vertical_accel_mps2 = [math]::Round(($samples.derived.vertical_accel_mps2 | ForEach-Object { [math]::Abs($_) } | Measure-Object -Maximum).Maximum, 3)
    max_angular_jerk_radps2 = [math]::Round(($samples.derived.angular_jerk_radps2 | Measure-Object -Maximum).Maximum, 3)
    max_trajectory_error_m = [math]::Round(($samples.derived.trajectory_max_m | Measure-Object -Maximum).Maximum, 3)
    raising_peak = [math]::Round(($samples.derived.raising_score | Measure-Object -Maximum).Maximum, 3)
    bumpiness_peak = [math]::Round(($samples.derived.bumpiness_score | Measure-Object -Maximum).Maximum, 3)
    sluggishness_peak = [math]::Round(($samples.derived.sluggish_score | Measure-Object -Maximum).Maximum, 3)
    oscillation_peak = [math]::Round(($samples.derived.oscillation_score | Measure-Object -Maximum).Maximum, 3)
  }
  recent_user_issues = $issues
}
$summary | ConvertTo-Json -Depth 8
