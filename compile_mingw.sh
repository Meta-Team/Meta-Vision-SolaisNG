#!/bin/sh
cmake -B build -DQt5_DIR=/lib/cmake/Qt5 -GNinja .
cmake --build build
