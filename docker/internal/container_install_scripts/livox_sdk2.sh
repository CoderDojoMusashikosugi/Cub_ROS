#!/bin/bash

apt update && apt install -y git cmake build-essential
cd /
git clone https://github.com/atinfinity/Livox-SDK2.git
cd ./Livox-SDK2
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release .. && make -j4
make install
ldconfig
cd /
rm -rf Livox-SDK2
