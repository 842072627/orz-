#!/bin/bash
export MSYS2_PATH_TYPE=win32
export PATH=/c/msys64/ucrt64/bin:$PATH
cd /c/Users/Administrator/Desktop/实验室/MAPF竞赛/LORR26_842072627
echo "[test] Running random test (200 steps)..."
./build/lifelong.exe -i example_problems/random.domain/random-example_400.json -s 200 -o build/test_output.json -c 2
echo "[test] Exit code: $?"