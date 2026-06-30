#!/bin/bash
set -eu
set -o pipefail

mkdir -p build
cmake -B build ./ \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON=OFF \
  -DENABLE_PYTHON_BINDINGS=ON \
  -DBUILD_TESTS=OFF \
  -DSUBMISSION_TRACK=AUTO
cmake --build build -j
