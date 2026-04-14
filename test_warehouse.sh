#!/bin/bash
# test_warehouse.sh - Run warehouse_small test
export MSYS2_PATH_TYPE=win32
export PATH=/c/msys64/ucrt64/bin:$PATH
cd /c/Users/Administrator/Desktop/实验室/MAPF竞赛/LORR26_842072627
echo "[test] Running warehouse_small test..."
./build/lifelong.exe -i example_problems/warehouse.domain/warehouse_small_200.json -s 1000 -o build/test_output.json
echo "[test] Exit code: $?"