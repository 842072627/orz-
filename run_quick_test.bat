@echo off
REM run_quick_test.bat - Build and test LORR v3.1.0 locally on Windows/MSYS2
REM Usage: Double-click or run from command line
REM Requirements: MSYS2 with ucrt64, pacman packages installed

setlocal
set PATH=C:\msys64\ucrt64\bin;%PATH%

echo [build] Cleaning and configuring...
cd /d C:\Users\Administrator\Desktop\实验室\MAPF竞赛\LORR26_842072627
rmdir /s /q build 2>nul
mkdir build
C:\msys64\ucrt64\bin\cmake.exe -B build -G "Unix Makefiles" . ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DMSYS2_BUILD=ON ^
    -DBoost_USE_STATIC_LIBS=OFF ^
    -DPYTHON_LIBRARY=C:/msys64/ucrt64/lib/libpython3.14.dll.a ^
    -DPYTHON_INCLUDE_DIR=C:/msys64/ucrt64/include/python3.14 ^
    -Wno-dev

echo [build] Compiling...
C:\msys64\ucrt64\bin\cmake.exe --build build -j4

echo.
echo [build] Done! Binary: build\lifelong.exe
echo.
echo [test] Running quick test (random map, 1000 steps)...
set PATH=C:\msys64\ucrt64\bin;%PATH%
build\lifelong.exe -i example_problems/random.domain/random-example_400.json -s 1000 -o build/test_output.json

echo.
echo [test] Results in build\test_output.json
pause