# Logger Module
A lightweight and customizable logging module for C++ applications. This module provides a flexible way to log messages with varying levels of severity, customize log formats, and measure execution times.

## Features
- Multiple Log Levels: Supports ERROR, WARNING, INFO, and DEBUG levels to categorize log messages.
- Customizable Log Functions: Allows users to define custom log formats for different log levels.
- File Logging: Option to enable logging to a file with automatic timestamped filenames.
- Stopwatch Utility: Built-in stopwatch to measure execution time intervals.
- Easy Integration: Simple and intuitive API for seamless integration into existing projects.

## Function decription

### Basic function
1. `e()` : Log error function. It will log data ta with flag ERROR. Logging Level is 1
2. `w()` : Log warning function. It will log data ta with flag WARNING. Logging Level is 2
3. `i()` : Log info function. It will log data ta with flag INFO. Logging Level is 3
4. `d()` : Log debug function. It will log data ta with flag DEBUG. Logging Level is 4

### Logging into file
1. `enableLogFile()` : Enable logging to file
2. `disaleLogFile()` : All data will be shown in console window
3. `logCSV()` : Log data into csv file. Columns name will be 
   1. `function's name`
   2. `line`
   3. Log data columns.

### Logging with timestamp
1. `reset()` : set clock to `0`
2. `timestamp()` : get timestamp from 1/1/1970 to now (default unit: nanosecond)
3. `s()` : start the stopwatch
4. `t()` : stop the stopwatch and return the duration since `s()` called

### Overwirte a log function (Comming soon)


## Example usage
Log data from several logger into only 1 file
```cpp
// initial logger 1
LogUtility log1;
log1.enableLogFile();

// log data
log1.e(VV"hello in error", "---1---\t\t", log1.timestamp());
log1.w(VV"hello in warning", "---2---\t", log1.timestamp());
log1.i(VV"hello in info", "---3---\t\t", log1.timestamp());
log1.d(VV"hello in debug", "---4---\t", log1.timestamp());

// initial logger 2
LogUtility log2;
log2.enableLogFile();

// log data
log2.e(VV"hello in error", "---5---\t\t", log2.timestamp());
log2.w(VV"hello in warning", "---6---\t", log2.timestamp());
log2.i(VV"hello in info", "---7---\t\t", log2.timestamp());
log2.d(VV"hello in debug", "---8---\t", log2.timestamp());
```

Basic example of logging into files along with the timestamp utility.
```cpp
int value = rand() % 100;
LogUtility log;
log.enableLogFile();
log.reset();
log.d(VV"time: ", log.timestamp(), "\tvalue: ", value);
```

The `timestamp()` function can return either a `uint64_t` or a `double`. By default, it returns a `uint64_t`. However, when the getDecimal flag is enabled, it returns a `double`. In this case, the user needs to cast the result to the appropriate type to obtain the correct value. Example is shown below:

```cpp
LogUtility log;
for (int i = 0; i < 100; ++i);
uint64_t temp = log.timestamp(MICRO, false, true);
double duration = *(double *)(&temp);
std::cout << "duration: " << duration << " microsecond\n";
```

In order to measure the duration bewteen functions called, we can use 2 command like below

```cpp
void callback(LogUtility *log, int ID) {
    (*log).i(VV "callback function in thread  ", ID, " is called after ", (*log).t(MILLI), " ms");
    (*log).s();
}
void threadFunc(int ID) {
    LogUtility log("log/", "thread", false);
    log.enableLogFile();
    for (int i = 0; i < 5; ++i) {
        callback(&log, ID);
        usleep(ID * 10000);
    }
}
void tc10() {
    // test call logger in thread    
    std::thread t1(threadFunc, 1);
    std::thread t2(threadFunc, 2);

    t1.join();
    t2.join();
}
```

