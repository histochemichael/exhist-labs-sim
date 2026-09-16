param([ValidateSet('loop','layout','hold','access','mobility')][string]$Mode='loop',[string]$Kind='quincy',[string]$Python='python')
$ErrorActionPreference='Stop'
Push-Location $PSScriptRoot
try {
    & $Python tools/unpack_assets.py
    if ($LASTEXITCODE -ne 0) { throw 'Asset unpack failed.' }
    if ($Mode -eq 'loop') { & $Python lab_rail_loop.py }
    elseif ($Mode -eq 'layout') { & $Python run_lab.py }
    elseif ($Mode -eq 'hold') { & $Python operate_lab.py }
    elseif ($Mode -eq 'access') { & $Python access_viewer.py --kind $Kind }
    elseif ($Mode -eq 'mobility') { & $Python mobile_base_bench.py --view }
    if ($LASTEXITCODE -ne 0) { throw 'Simulator exited with an error.' }
} finally { Pop-Location }
