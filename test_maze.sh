#!/bin/bash
# test_maze.sh - Run maze test
export MSYS2_PATH_TYPE=win32
export PATH=/c/msys64/ucrt64/bin:$PATH
cd /c/Users/Administrator/Desktop/实验室/MAPF竞赛/LORR26_842072627
echo "[test] Running maze test..."
./build/lifelong.exe -i example_problems/maze.domain/maze-example_40.json -s 100 -o build/maze_output.json -c 2
echo "[test] Exit code: $?"