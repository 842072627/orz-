@echo off
set PATH=C:\msys64\ucrt64\bin;%PATH%
cd /d C:\Users\Administrator\Desktop\实验室\MAPF竞赛\LORR26_842072627
build\lifelong.exe -i example_problems\warehouse.domain\warehouse_small_200.json -s 1000 -o build\test_output.json
echo Exit code: %ERRORLEVEL%