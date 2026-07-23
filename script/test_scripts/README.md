# GTEST for UT/IT
- [QuickStart](#1-quickstart)
- [Build and Run for Target Test Case](#2-build-and-run-for-target-test-case)
# Layout
```
adas_service
├── adas_main
│   └── ...
├── ...
│   ├── ...
│   ├── test_ut <---------------------------------- Unit Test
│   │   ├── <test-project> <----------------------- Test Project
│   │   |   ├── test_<test-project>.cpp <---------- Test scripts
|   |   |   ├── test_<test-project>.h <------------ Header Files
│   │   |   └── CMakeLists.txt <------------------- Configure Sub Directories
│   │   ├── <test-project> <----------------------- Test Project
│   │   |   ├── test_<test-project>.cpp <---------- Test scripts
|   |   |   ├── test_<test-project>.h <------------ Header Files
│   │   |   └── CMakeLists.txt <------------------- Configure Sub Directories
|   |   ├── build <-------------------------------- After Building CMakeLists ROOT
|   |   └── CMakeLists.txt <----------------------- Configuration ROOT
│   └── test_it <---------------------------------- Integration Test
│       ├── <test-project> <----------------------- Test Project
│       |   ├── test_<test-project>.cpp <---------- Test scripts
|       |   ├── test_<test-project>.h <------------ Header Files
│       |   └── CMakeLists.txt <------------------- Configure Sub Directories
│       ├── <test-project> <----------------------- Test Project
│       |   ├── test_<test-project>.cpp <---------- Test scripts
|       |   ├── test_<test-project>.h <------------ Header Files
│       |   └── CMakeLists.txt <------------------- Configure Sub Directories
|       ├── build <-------------------------------- After Building CMakeLists ROOT
|       └── CMakeLists.txt <----------------------- Configuration ROOT
└── scripts
    └── test_scripts
        ├── run_test_scripts.sh <------------------ Run Test Scripts
        └── README.md
 
```
# 1. QuickStart

Build ADAS SDK for PC

```
./script/pc_build.sh --test -j4
```

Build for Unit Test

```
./script/test_scripts/run_test_scripts.sh --mode=all --type=unit-test
```

Build for Integration Test

```
./script/test_scripts/run_test_scripts.sh --mode=all --type=integration-test
```

Generate Coverage Report
```
./script/test_scripts/generate_test_scripts.sh
```

# 2. Build and Run for Target Test Case
## 2.1. Options
```shell
--mode=MODE
    Set the mode of operation (default: all).
    Valid options:
        build   - to build the project
        run     - to run the project
        all     - to build and run the entire project

--type=TYPE
    Set the type of tests to execute (default: unit-test).
    Valid options:
        unit-test           - to run unit tests
        integration-test    - to run integration tests

--target_src=FILE/FOLDER
    Specify the test project file to run.

--help
    Display the help message.
```
> If you use the **target_src** argument for build, you must add **a name of directory/folder** that includes test_ut/test_it.
## 2.2. Usage 
Build and run entire tests:
```
./script/test_scripts/run_test_scripts.sh --mode=all --type=<TYPE>
```

Build for target source

```
./script/test_scripts/run_test_scripts.sh --mode=<MODE> --type=<TYPE> --target_src=<TARGET_SRC>
```
`<TARGET_SRC>`: is a name of directory that includes the test_ut/test_it directory.

Example:

Build a unit test in the control (`<test-project>`) of the Adas service:

```
./script/test_scripts/run_test_scripts.sh --mode=build --type=unit-test --target_src=control
```

Run a unit test in the control (`<test-project>`) of the Adas service:

```
./script/test_scripts/run_test_scripts.sh --mode=run --type=unit-test --target_src=control
```