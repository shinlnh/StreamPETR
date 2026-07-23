source /opt/ros/foxy/setup.bash
mkdir -p build
cd build
colcon build --base-path ../../../../ipc_helper/src
source install/setup.bash
cmake ..
make -j$(nproc) -l$(nproc)