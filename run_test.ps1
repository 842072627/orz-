# run_test.ps1
param(
    [string]$name = "test",
    [int]$steps = 3000
)

$logDir = "C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627"
$env:Path = "C:\msys64\ucrt64\bin;$env:Path"
$outputFile = "$logDir\${name}.json"

$start = Get-Date
$startStr = $start.ToString("HH:mm:ss")
Write-Host "=== TEST $name START: $startStr ==="

cd "C:\Users\Administrator\Desktop\test-studio-mapf\LORR26_842072627"
.\build\lifelong.exe -i "C:\Users\Administrator\Desktop\test-studio-mapf\big-final-test\game.domain\orz-example_4000.json" -s $steps -c 1 -o $outputFile -n 120000 -t 1000

$end = Get-Date
$endStr = $end.ToString("HH:mm:ss")
$duration = ($end - $start).TotalSeconds
Write-Host "=== TEST $name END: $endStr (${duration}s) ==="

python -c "
import json
d = json.load(open('$outputFile'))
print('Tasks: %d' % d.get('numTaskFinished'))
" 2>&1
