#!/usr/bin/env sh
# Host simulation of the real detector.cpp / cycle.cpp against synthetic pot profiles.
# Needs any C++17 compiler. With no compiler installed:  python -m pip install ziglang
# then run with CXX="python -m ziglang c++" ./run.sh
set -e
cd "$(dirname "$0")"
CXX=${CXX:-g++}
$CXX -std=c++17 -O1 -Wall -Ifake -I../.. sim.cpp spi_stub.cpp ../../detector.cpp ../../cycle.cpp ../../ADXL.cpp -o sim.exe
./sim.exe
