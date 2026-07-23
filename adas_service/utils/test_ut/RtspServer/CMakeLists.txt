# Get the directory name
get_filename_component(TEST_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

# ====== 1. SETUP VARIABLE SECTION ======
set(RtspServer
    ${ADAS_SDK_ROOT}/common/communications/RtspServer.cpp
)

# ====== 2. FIND PACKAGE SECTION ======
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-rtp-1.0)
pkg_check_modules(GST_RTSP_SERVER REQUIRED gstreamer-rtsp-server-1.0)
# ====== 3. ADD INCLUDE DIRECTORIES ======
# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GST_INCLUDE_DIRS}
    ${GST_RTSP_SERVER_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${ADAS_SERVICE_ROOT}
    ${ADAS_SDK_ROOT}
    ${ADAS_SERVICE_ROOT}/adas_main
    ${ADAS_SERVICE_ROOT}/utils
    ${ADAS_SDK_ROOT}/common/communications
    ${ADAS_SDK_ROOT}/common/ipc_helper/
    ${ADAS_SDK_ROOT}/common/src/inc
)

#add_definitions(${GST_CFLAGS_OTHER} ${GST_RTSP_SERVER_CFLAGS_OTHER})
# ====== 4. ADD EXECUTABLE ======
# Add the executable for statevehicle
file(GLOB TEST_SOURCE "./*cpp")
add_executable(
                ${TEST_NAME} 
                ${TEST_SOURCE}
                ${RtspServer}
)

# ====== 5. ADD COMPILER OPTIONS ======
target_compile_definitions(${TEST_NAME} PRIVATE -DUT_TEST)

# ====== 6. LINK EXECUTABLE ======  
# Link the executable with gtest
target_link_libraries(
    ${TEST_NAME} 
    ${GTEST_LIBRARIES}
    ${Eigen3_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${GST_LIBRARIES}
    ${GST_RTSP_SERVER_LIBRARIES}
    ${GST_RTP_LIBRARIES}
)
