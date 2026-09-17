param([ValidateSet('loop','layout','hold','access','mobility')][string]$Mode='loop',[string]$Kind='quincy',[string]$Python='python')
$ErrorActionPreference='Stop'
& (Join-Path $PSScriptRoot 'simulation/launch.ps1') -Mode $Mode -Kind $Kind -Python $Python
