@echo off
chcp 65001 >nul 2>&1
setlocal

set "PATH=C:\msys64\ucrt64\bin;%PATH%"
set "PROJ_DIR=C:\Users\Administrator\Desktop\实验室\MAPF竞赛\LORR26_842072627"

echo [build] Configuring...
if not exist "%PROJ_DIR%\build\lifelong.exe" (
    rmdir /s /q "%PROJ_DIR%\build" 2>nul
    mkdir "%PROJ_DIR%\build"
    C:\msys64\ucrt64\bin\cmake.exe -B "%PROJ_DIR%\build" -G "Unix Makefiles" "%PROJ_DIR%" -DCMAKE_BUILD_TYPE=Release -DMSYS2_BUILD=ON -DBoost_USE_STATIC_LIBS=OFF -DPYTHON_LIBRARY=C:/msys64/ucrt64/lib/libpython3.14.dll.a -DPYTHON_INCLUDE_DIR=C:/msys64/ucrt64/include/python3.14 -Wno-dev >nul 2>&1
    echo [build] Compiling (this may take a few minutes)...
    C:\msys64\ucrt64\bin\cmake.exe --build "%PROJ_DIR%\build" -j4 >nul 2>&1
)

echo [build] Done. Running tests via PowerShell...
powershell.exe -ExecutionPolicy Bypass -File "%~dp0run_parallel_quick_test.ps1"

pause