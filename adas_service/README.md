# ADAS service


- [ADAS service](#adas-service)
  - [Build for the PC](#build-adas_service-for-pc)
    - [Quick start](#quick-start)
    - [Build and Run (Step by Step)](#build-and-run-step-by-step)
      - [1. Create folder `pc_build `](#1-create-folder-pc_build-)
      - [2. Source necessary file to `pc_build` and build SDK](#2-source-necessary-file-to-pc_build-and-build-sdk)
      - [3. After compiled, source before run the ADAS app](#3-after-compiled-source-before-run-the-adas-app)
  - [Build for the target board](#build-adas_service-for-the-target-board)
    - [Quick start](#quick-start-1)
    - [Build and Run (Step by Step)](#build-and-run-step-by-step-1)
      - [1. Create folder `build`](#1-create-folder-build)
      - [2. Source necessary file to `build` and build SDK](#2-source-necessary-file-to-build-and-build-sdk)
      - [3. After compiled, source before run the ADAS app](#3-after-compiled-source-before-run-the-adas-app-1)

## Build for PC
### Quick start
Run the command at the top level of the workspace

Build command:
```
./script/pc_build.sh -j 4
```

Run application command:
```
./script/run_adas.sh
```

### Build and Run (Step by Step)
#### 1. Create folder `pc_build `
Make a `pc_build` directory in the parent directory of this README.md file's directory, the folder structure after then should look like this:
```
adas_sdk
├── adas_service
│   └── README.md  <------------ right now README.md file
├── pc_build       <------------ the pc_build directory you've just made
├── common
├── script
├── CMakeLists.txt
└── README.md
```
```
mkdir -p pc_build
```
#### 2. Source necessary file to `pc_build` and build SDK
```
source script/pc_setenv.sh
colcon build \
        --base-paths /<path-to-sdk>/ros_bridge_support_ws/src /<path-to-sdk>/common/ipc_helper/src \
        --packages-select carla_msgs_custom ipc_helper
source install/setup.bash
cd pc_build
cmake ..
make -j4
```
#### 3. After compiled, source before run the ADAS app
```
source ../script/setup_adas_service_runtime_env.bash
./pc_build/adas_service/adas_service --src_type=udp
```

> Running option argument `--help` for more informations.

## Build for the target board
### Quick start
Run the command at the top level of the workspace

Build command:
```
./script/build.sh -j 4 -m dds               # Build with DDS mode
./script/build.sh -j 4 -m can               # Build with CAN mode
```

Run commands on target board for adas_service
```
./script/run_adas.sh                        # Run on ADAS board with DDS mode
```
### Build and Run (Step by Step)
#### 1. Create folder `build`
Make a build directory in the parent directory of this README.md file's directory, the folder structure after then should look like this:
```
adas_sdk
├── adas_service
│   └── README.md  <------------  right now README.md file
├── build       <---------------  the build directory you've just made
├── common
├── script
├── CMakeLists.txt
└── README.md
```
```
mkdir -p build
```
#### 2. Source necessary file to `build` and build SDK
Inside the `adas_sdk/build` directory, BEFORE run cmake, you MUST execute `source` command to specify the SDK of the target board.

Build with DDS mode:
```
source script/setenv_nativesdk.sh
colcon build \
        --base-paths /<path-to-sdk>/ros_bridge_support_ws/src /<path-to-sdk>/common/ipc_helper/src \
        --cmake-args -DCMAKE_TOOLCHAIN_FILE=/<path-to-sdk>/toolchain.cmake \
        --packages-select carla_msgs_custom ipc_helper
sed -i "s|${SDKTARGETSYSROOT}||g" install/setup.bash
sed -i "s|${SDKTARGETSYSROOT}||g" install/setup.sh
cd build
cmake ..  -Dipc_helper_DIR=/<path-to-sdk>/install/ipc_helper/share/ipc_helper/cmake \
          -Dcarla_msgs_custom_DIR=/<path-to-sdk>/install/carla_msgs_custom/share/carla_msgs_custom/cmake
make -j4
```

Build with CAN mode:
```
source script/setenv_nativesdk.sh
colcon build \
        --base-paths /<path-to-sdk>/ros_bridge_support_ws/src /<path-to-sdk>/common/ipc_helper/src \
        --cmake-args -DCMAKE_TOOLCHAIN_FILE=/<path-to-sdk>/toolchain.cmake \
        --packages-select carla_msgs_custom ipc_helper
sed -i "s|${SDKTARGETSYSROOT}||g" install/setup.bash
sed -i "s|${SDKTARGETSYSROOT}||g" install/setup.sh
cd build
cmake ..  -Dipc_helper_DIR=/<path-to-sdk>/install/ipc_helper/share/ipc_helper/cmake \
          -Dcarla_msgs_custom_DIR=/<path-to-sdk>/install/carla_msgs_custom/share/carla_msgs_custom/cmake \
          -DENABLE_CAN=ON
make -j4
```

#### 3. After compiled, source before run the ADAS app
You can either run on PC for functional testing, or run on the Jetson board for accurate performance. 

```
# Source for DDS mode
source ../script/setup_adas_service_runtime_env.bash -m dds  

# Source for CAN mode
source ../script/setup_adas_service_runtime_env.bash -m can

# Run adas_service
./build/adas_service/adas_service --src_type=udp
```
> Running option argument `--help` for more informations.