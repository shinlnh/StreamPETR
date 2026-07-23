# Cuda and OpenCV
export PATH=/usr/local/cuda-11.1/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.1/lib64:$LD_LIBRARY_PATH
export PATH="/opt/Qt/5.15.2/gcc_64/bin/:$PATH"
export PC_BUILD=true

# OpenDDS configure script: ./configure
export ACE_ROOT=/prj/es-automotive/adas/common/OpenDDS-3.14/ACE_wrappers
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/prj/es-automotive/adas/common/OpenDDS-3.14/ACE_wrappers/lib:/prj/es-automotive/adas/common/OpenDDS-3.14/lib
export DANCE_ROOT=unused
export MPC_ROOT=/prj/es-automotive/adas/common/OpenDDS-3.14/ACE_wrappers/MPC
export PATH=${PATH}:/prj/es-automotive/adas/common/OpenDDS-3.14/ACE_wrappers/bin:/prj/es-automotive/adas/common/OpenDDS-3.14/bin
export DDS_ROOT=/prj/es-automotive/adas/common/OpenDDS-3.14
export TAO_ROOT=/prj/es-automotive/adas/common/OpenDDS-3.14/ACE_wrappers/TAO
export CIAO_ROOT=unused
export RAPIDJSON_ROOT=/prj/es-automotive/adas/common/OpenDDS-3.14/tools/rapidjson