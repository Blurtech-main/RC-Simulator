param([switch]$RunTests)
$ErrorActionPreference = 'Stop'
$vs = & 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe' -latest -products * -property installationPath
if (-not $vs) { throw 'Visual Studio Build Tools were not found.' }
$cmake = Join-Path $vs 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
if (-not (Test-Path -LiteralPath $cmake)) { $cmake = (Get-Command cmake).Source }
& $cmake -S . -B build -G 'Visual Studio 18 2026' -A x64
if ($LASTEXITCODE) { exit $LASTEXITCODE }
& $cmake --build build --config Release
if ($LASTEXITCODE) { exit $LASTEXITCODE }
if ($RunTests) {
  & '.\build\Release\RCTrainer.exe' --self-test
  exit $LASTEXITCODE
}

