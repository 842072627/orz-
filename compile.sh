#!/bin/bash
set -eu
set -o pipefail

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j8
