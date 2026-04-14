$ErrorActionPreference = "Continue"
$oldPath = $env:PATH
$env:PATH = "C:\msys64\ucrt64\bin;" + $env:PATH
$projDir = "C:\Users\Administrator\Desktop\实验室\MAPF竞赛\LORR26_842072627"
Set-Location $projDir
$exe = "C:\Users\Administrator\Desktop\实验室\MAPF竞赛\LORR26_842072627\build\lifelong.exe"
$arg = @("-i", "example_problems\warehouse.domain\warehouse_small_200.json", "-s", "1000", "-o", "build\test_output.json")
Write-Host "[run_test] Starting lifelong.exe..."
$proc = Start-Process -FilePath $exe -ArgumentList $arg -PassThru -Wait -NoNewWindow -RedirectStandardOutput "$projDir\build\stdout.txt" -RedirectStandardError "$projDir\build\stderr.txt"
Write-Host "[run_test] Exit code:" $proc.ExitCode
$env:PATH = $oldPath