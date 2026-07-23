#ifndef DEBUG_TO_DUMPLOG_H
#define DEBUG_TO_DUMPLOG_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdarg>
#include "common.h"

// Platform-specific directory creation
#if defined(_WIN32) || defined(_WIN64)
#include <direct.h>
#define CREATE_DIR(path) _mkdir(path)
#else
#include <sys/stat.h>
#include <sys/types.h>
#define CREATE_DIR(path) mkdir(path, 0755)
#endif

// Helper function to get the name of the function
static inline std::string get_function_name(const char* function) {
    return std::string(function);
}

// Helper function to format log messages
static inline std::string format_log_message(const char* format, ...) {
    std::ostringstream oss;
    va_list args;
    va_start(args, format);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    oss << buffer;
    return oss.str();
}

// Directory for log files
const std::string log_directory = "log";

// Check if DEBUG is already defined before redefining it
#if LOG_LEVEL >= LOG_LEVEL_DEBUG

#undef DEBUG

#define DEBUG(...) do { \
    std::cout << "[DEBUG][" << __FUNCTION__ << ":" << __LINE__ << "]: " << format_log_message(__VA_ARGS__) << std::endl; \
    std::string function_name = get_function_name(__FUNCTION__); \
    int line_number = __LINE__; \
    std::string log_path = log_directory; \
    if (CREATE_DIR(log_path.c_str()) != 0 && errno != EEXIST) { \
        ERROR("Error creating log directory: %s", strerror(errno));\
    } \
    std::ofstream log_file(log_path + "/" + function_name + ".txt", std::ios::app); \
    if (log_file.is_open()) { \
        log_file << "[DEBUG][" << function_name << "][Line " << line_number << "]: " << format_log_message(__VA_ARGS__) << std::endl; \
        log_file.close(); \
    } \
} while (0)

#endif // DEBUG

#endif // DEBUG_TO_DUMPLOG_H
