#!/bin/sh
cmake -B build -DQt5_DIR="/usr/include/x86_64-linux-gnu/qt5/" -GNinja .
cmake --build build
