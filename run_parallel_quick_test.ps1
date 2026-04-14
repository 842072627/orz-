# run_parallel_quick_test.ps1 - Called by run_parallel_quick_test.bat
$env:Path = "C:\msys64\ucrt64\bin;C:\msys64\usr\bin;" + $env:Path
$ProjectDir = "C:\Users\Administrator\Desktop\实验室\MAPF竞赛\LORR26_842072627"
$ArchiveDir = "C:\Users\Administrator\Desktop\实验室\MAPF竞赛\archive\rounds2"
$Exe = "$ProjectDir\build\lifelong.exe"
$MaxSteps = 1000

# Find next round
$round = 1
while (Test-Path "$ArchiveDir\round$round") { $round++ }
$RoundDir = "$ArchiveDir\round$round"
New-Item -ItemType Directory -Path $RoundDir -Force | Out-Null

$Maps = @(
    @{Name="bos"; Path="example_problems/city.domain/bos-example_600.json"},
    @{Name="fulfill-A"; Path="example_problems/warehouse.domain/fulfill-example_2500.json"},
    @{Name="fulfill-B"; Path="example_problems/warehouse.domain/sortation_large_2000.json"},
    @{Name="fulfill-C"; Path="example_problems/warehouse.domain/warehouse_small_200.json"},
    @{Name="iron"; Path="example_problems/iron_harvest.domain/iron-example_10000.json"},
    @{Name="maze-A"; Path="example_problems/maze.domain/maze-example_40.json"},
    @{Name="maze-B"; Path="example_problems/random.domain/random_32_32_20_100.json"},
    @{Name="orz"; Path="example_problems/game.domain/orz-example_1800.json"},
    @{Name="rand-A"; Path="example_problems/random.domain/random-example_400.json"},
    @{Name="room-A"; Path="example_problems/room.domain/room-example_150.json"},
    @{Name="room-B"; Path="example_problems/warehouse.domain/warehouse_large_5000.json"}
)

Write-Host "============================================"
Write-Host "  Round $round - Quick Test ($MaxSteps steps)"
Write-Host "  Output: $RoundDir"
Write-Host "============================================"

$results = @{}

foreach ($map in $Maps) {
    $json = "$RoundDir\$($map.Name)_output.json"
    Write-Host "Testing $($map.Name)..."
    $errLog = "$env:TEMP\lifelong_err_$($map.Name).log"
    $proc = Start-Process -FilePath $Exe -ArgumentList "-i `"$($map.Path)`" -o `"$json`" -s $MaxSteps" -Wait -PassThru -NoNewWindow -RedirectStandardOutput $errLog
    if ((Test-Path $json) -and (Get-Content $json -Raw -ErrorAction SilentlyContinue)) {
        try {
            $c = Get-Content $json -Raw | ConvertFrom-Json
            $t = $c.numTaskFinished
            $m = $c.makespan
            if ($m -and $m -gt 0) {
                $s = [math]::Round($t / $m, 3)
                $results[$map.Name] = $s
                Write-Host "  -> $($map.Name): $t/$m = $s"
            } else {
                $results[$map.Name] = "N/A"
                Write-Host "  -> $($map.Name): N/A (makespan=0)"
            }
        } catch {
            $results[$map.Name] = "ERR"
            Write-Host "  -> $($map.Name): ERROR"
        }
    } else {
        $results[$map.Name] = "FAIL"
        Write-Host "  -> $($map.Name): FAILED (exit=$($proc.ExitCode))"
    }
}

Write-Host "============================================"
Write-Host "Results saved to: $RoundDir"

# Save results.txt
$out = "Round $round - Quick Test ($MaxSteps steps)`n"
$out += "Saved: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')`n`n"
foreach ($map in $Maps) {
    $s = $results[$map.Name]
    $out += "{0,-12} {1}`n" -f $map.Name, $s
}
$out += "`nBUILD_SUCCESS`n"
Set-Content "$RoundDir\results.txt" $out -Encoding UTF8
"BUILD_SUCCESS" | Set-Content "$RoundDir\BUILD_STATUS"

# Copy source files
Copy-Item "$ProjectDir\src\MAPFPlanner.cpp" "$RoundDir\" -ErrorAction SilentlyContinue
Copy-Item "$ProjectDir\src\TaskScheduler.cpp" "$RoundDir\" -ErrorAction SilentlyContinue
Copy-Item "$ProjectDir\src\Executor.cpp" "$RoundDir\" -ErrorAction SilentlyContinue
Copy-Item "$ProjectDir\src\Entry.cpp" "$RoundDir\" -ErrorAction SilentlyContinue

# Output results for parsing
Write-Host "==RESULTS=="
foreach ($map in $Maps) {
    Write-Host "$($map.Name)=$($results[$map.Name])"
}