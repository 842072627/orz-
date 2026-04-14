$env:Path = "C:\msys64\ucrt64\bin;C:\msys64\usr\bin;" + $env:Path
Set-Location "C:\Users\Administrator\Desktop\实验室\MAPF竞赛\LORR26_842072627"
$ErrorActionPreference = "SilentlyContinue"

$ArchiveDir = "C:\Users\Administrator\Desktop\实验室\MAPF竞赛\archive\rounds2"
$MaxSteps = 1000
$Round = 1
while (Test-Path "$ArchiveDir\round$Round") { $Round++ }
$RoundDir = "$ArchiveDir\round$Round"
New-Item -ItemType Directory -Path $RoundDir -Force | Out-Null

$maps = @(
    @{n="random"; p="example_problems/random.domain/random-example_400.json"},
    @{n="warehouse_small"; p="example_problems/warehouse.domain/warehouse_small_200.json"},
    @{n="sortation_large"; p="example_problems/warehouse.domain/sortation_large_2000.json"},
    @{n="warehouse_large"; p="example_problems/warehouse.domain/warehouse_large_5000.json"},
    @{n="paris"; p="example_problems/city.domain/bos-example_600.json"},
    @{n="brc202d"; p="example_problems/game.domain/orz-example_1800.json"}
)

Write-Host "============================================"
Write-Host "  Round $Round - Quick Test ($MaxSteps steps)"
Write-Host "  Output: $RoundDir"
Write-Host "============================================"

$totalScore = 0
$results = @()

foreach ($m in $maps) {
    $json = "$RoundDir\$($m.n)_output.json"
    Write-Host "Testing $($m.n)..."
    $proc = Start-Process -FilePath ".\build\lifelong.exe" -ArgumentList "-i `"$($m.p)`" -o `"$json`" -s $MaxSteps" -Wait -PassThru -NoNewWindow -RedirectStandardOutput "$env:TEMP\out_$($m.n).log"
    if (Test-Path $json) {
        $c = Get-Content $json -Raw | ConvertFrom-Json
        $t = $c.numTaskFinished
        $mk = $c.makespan
        $s = [math]::Round($t / $mk, 3)
        $p = $c.numPlannerErrors; $se = $c.numScheduleErrors; $to = $c.numEntryTimeouts
        $line = $p + $se + $to
        $warn = if ($line -gt 0) { "${p}P/${se}S/${to}T" } else { "None" }
        Write-Host "  -> $($m.n): $t tasks, score=$s [$warn]"
        $totalScore += $s
        $results += ,@{Name=$m.n; Tasks=$t; Score=$s; Line=$line}
    } else {
        Write-Host "  -> $($m.n): FAILED (exit=$($proc.ExitCode))"
        $results += ,@{Name=$m.n; Tasks="FAIL"; Score="-"; Line=0}
    }
}

Write-Host "============================================"
Write-Host "Total score=$([math]::Round($totalScore,3))"
Write-Host "============================================"

# Save results
$out = "Round $Round - Quick Test ($MaxSteps steps)`nSaved: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')`n`n"
$out += "Map                       Tasks   Score   Line`n----                       -----   -----   ----`n"
foreach ($r in $results) { $out += "{0,-26} {1,6} {2,7} {3,6}`n" -f $r.Name, $r.Tasks, $r.Score, $r.Line }
$out += "----                       -----   -----   ----`n"
$out += "{0,-26} {1,6} {2,7}`n" -f "Total", "", ([math]::Round($totalScore,3))
Set-Content "$RoundDir\results.txt" $out -Encoding UTF8
"BUILD_SUCCESS" | Set-Content "$RoundDir\BUILD_STATUS"
Copy-Item "src\MAPFPlanner.cpp" "$RoundDir\" -ErrorAction SilentlyContinue
Copy-Item "src\TaskScheduler.cpp" "$RoundDir\" -ErrorAction SilentlyContinue
Copy-Item "src\Executor.cpp" "$RoundDir\" -ErrorAction SilentlyContinue
Copy-Item "src\Entry.cpp" "$RoundDir\" -ErrorAction SilentlyContinue
Write-Host "Done"