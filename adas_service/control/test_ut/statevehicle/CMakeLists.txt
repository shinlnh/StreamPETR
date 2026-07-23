# Get the directory name
get_filename_component(TEST_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

# ====== 1. SETUP VARIABLE SECTION ======
set(statevehicle
    ${ADAS_SERVICE_ROOT}/control/statevehicle.cpp
)
# ====== 2. FIND PACKAGE SECTION ======

# ====== 3. ADD INCLUDE DIRECTORIES ======
# Include directories
include_directories(
    ${ADAS_SERVICE_ROOT}/control
)

# ====== 4. ADD EXECUTABLE ======
# Add the executable for statevehicle
add_executable(${TEST_NAME} 
                test_statevehicle.cpp 
                ${statevehicle}
)

# ====== 5. ADD COMPILER OPTIONS ======
target_compile_definitions(${TEST_NAME} PRIVATE -DUT_TEST)

# ====== 6. LINK EXECUTABLE ======  
# Link the executable with gtest
target_link_libraries(${TEST_NAME} 
                        ${GTEST_LIBRARIES}
)

# ====== 7. ENABLE and ADDTESTING ======
# # Enable testing
# enable_testing()
# # Add the test
# add_test(NAME ${TEST_NAME} 
#             COMMAND ${TEST_NAME}
# )
