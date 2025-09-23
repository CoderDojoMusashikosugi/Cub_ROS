apt -y install qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev cmake
git clone --recurse-submodules https://github.com/BehaviorTree/Groot.git
cd Groot
cmake -S . -B build
cmake --build build -j$(($(nproc)-1))
cmake --install build
rm -r build

echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >> /etc/user.bashrc
