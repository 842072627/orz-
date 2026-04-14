#!/bin/bash
# compile.sh - Local MSYS2 build script for LORR v3.1.0
# Usage: ./compile.sh (or double-click run_quick_test.bat on Windows)
#
# Requirements:
#   - MSYS2 with ucrt64 environment
#   - pacman -S mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-boost
#            mingw-w64-ucrt-x86_64-pybind11 mingw-w64-ucrt-x86_64-cmake ninja

set -e
cd "$(dirname "$0")"
PROJECT_DIR="$(pwd)"
BUILD_DIR="$PROJECT_DIR/build"

echo "[compile] Cleaning build directory..."
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

echo "[compile] Configuring with CMake (MSYS Makefiles, dynamic Boost linking)..."
C:/msys64/ucrt64/bin/cmake.exe -B "$BUILD_DIR" -G "Unix Makefiles" . \
    -DCMAKE_BUILD_TYPE=Release \
    -DMSYS2_BUILD=ON \
    -DBoost_USE_STATIC_LIBS=OFF

echo "[compile] Building lifelong.exe..."
C:/msys64/ucrt64/bin/cmake.exe --build "$BUILD_DIR" -j4

echo "[compile] Done! Binary: $BUILD_DIR/lifelong.exe"