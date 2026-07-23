set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_CROSSCOMPILING TRUE)
set(CMAKE_SYSROOT $ENV{SDKTARGETSYSROOT})

# Set compiler (FSL SDK)
set(CMAKE_C_COMPILER aarch64-fsl-linux-gcc)
set(CMAKE_CXX_COMPILER aarch64-fsl-linux-g++)

# Force use native Python for build tools
# This fixes the issue where CMake finds ARM Python instead of x86_64 Python
set(PYTHON_EXECUTABLE $ENV{OECORE_NATIVE_SYSROOT}/usr/bin/python3 CACHE FILEPATH "Python executable" FORCE)
set(PythonInterp_FOUND TRUE CACHE BOOL "Python interpreter found" FORCE)
set(PYTHON_VERSION_STRING "3.8" CACHE STRING "Python version" FORCE)
set(PYTHON_VERSION_MAJOR "3" CACHE STRING "Python major version" FORCE)
set(PYTHON_VERSION_MINOR "8" CACHE STRING "Python minor version" FORCE)

# Set path for cmake
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
