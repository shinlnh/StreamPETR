#ifndef GENERAL_LOGGER_H
#define GENERAL_LOGGER_H

#include <iostream>
#include <functional>
#include <fstream>
#include <vector>
#include <cstdarg>
#include <sstream>
#include <unistd.h>
#include <mutex>
#include <unordered_map>

#include "../common.h"
#include "../stopwatch.h"

#define VV         std::string("[") + __FUNCTION__ + ":" + std::to_string(__LINE__) + "]: ",
#define LOG_EXTENT ".log"

/*------------------------------------default function for logger------------------------------------*/
template<typename... Args>
std::string concat(std::string separate = "", const Args&... args)
{
    std::ostringstream oss;
    int expander[] = { 0, (oss << args << separate, 0)... }; // trick lord
    static_cast<void>(expander);                             // Avoid unused variable warning
    return oss.str();
}

template<typename... Args>
std::string defaultError(const Args&... args)
{
    std::ostringstream msg;
    msg << "[ERROR]: " << concat("", args...) << std::endl;
    return msg.str();
}

template<typename... Args>
std::string defaultWarning(const Args&... args)
{
    std::ostringstream msg;
    msg << "[WARNING]: " << concat("", args...) << std::endl;
    return msg.str();
}

template<typename... Args>
std::string defaultInfo(const Args&... args)
{
    std::ostringstream msg;
    msg << "[INFO]: " << concat("", args...) << std::endl;
    return msg.str();
}

template<typename... Args>
std::string defaultDebug(const Args&... args)
{
    std::ostringstream msg;
    msg << "[DEBUG]: " << concat("", args...) << std::endl;
    return msg.str();
}
/*---------------------------------------------------------------------------------------------------*/

class LogUtility
{
protected:
    // Atributes
    Stopwatch clock;
    std::string filename, path_to_logfolder;
    bool log_file;
    static std::unordered_map<std::string, std::mutex> locks;

    // Internal function
    static std::unordered_map<std::string, std::mutex>& getLocks()
    {
        static std::unordered_map<std::string, std::mutex> locks;
        return locks;
    }
    std::string generateFilename(std::string filename = "", bool untilSec = false)
    {
        // generate a name with format:
        // <year_<month>_<day>_<hr>-<min>-<sec>
        std::ostringstream oss;
        oss << clock.getTime(untilSec) << filename << LOG_EXTENT;
        return oss.str();
    }
    void makeLogDir(std::string folder_path = "log/")
    {
        path_to_logfolder = folder_path;
        std::ostringstream oss;
        oss << "mkdir -p " << folder_path;
        if (system(oss.str().c_str()) != 0) {
            std::cout << "Make log directory unsuccessfully" << std::endl;
            return;
        }
    }
    void writeFile(std::string data)
    {
        std::lock_guard<std::mutex> lock(getLocks()[filename]);
        std::ofstream file;
        file.open(filename, std::ios::app);
        if (!file.is_open()) {
            // std::cout << "Cannot open " << filename << std::endl;
            return;
        }
        file << data;
        file.close();
    }

public:
    LogUtility(std::string folder_log = "log/", std::string filename = "", bool untilSec = false)
    {
        log_file = false;
        path_to_logfolder = folder_log;
        makeLogDir(path_to_logfolder);
        this->filename = concat("", folder_log, generateFilename(filename, untilSec));
    }
    ~LogUtility()
    {
        if (LOG_LEVEL > 0 && log_file) {
            // const size_t buf_size = 1024;
            // char buffer[buf_size];
            // if (getcwd(buffer, buf_size))
            //     std::cout << "Path to folder contain log files is " << buffer << "/" << path_to_logfolder <<
            //     std::endl;
        }
    }
    /*-----------------------------------------Basic log function-----------------------------------------*/
    // log error
    template<typename... Args>
    void e(const Args&... args)
    {
        if (LOG_LEVEL < LOG_LEVEL_ERROR) {
            // do nothing
            return;
        }
        std::string msg = defaultError(args...);
        if (log_file) {
            writeFile(msg);
            return;
        }
        std::cout << msg;
    }

    // log warning
    template<typename... Args>
    void w(const Args&... args)
    {
        if (LOG_LEVEL < LOG_LEVEL_WARN) {
            // do nothing
            return;
        }
        std::string msg = defaultWarning(args...);
        if (log_file) {
            writeFile(msg);
            return;
        }
        std::cout << msg;
    }

    // log information
    template<typename... Args>
    void i(const Args&... args)
    {
        if (LOG_LEVEL < LOG_LEVEL_INFO) {
            // do nothing
            return;
        }
        std::string msg = defaultInfo(args...);
        if (log_file) {
            writeFile(msg);
            return;
        }
        std::cout << msg;
    }

    // log debug
    template<typename... Args>
    void d(const Args&... args)
    {
        if (LOG_LEVEL < LOG_LEVEL_DEBUG) {
            // do nothing
            return;
        }
        std::string msg = defaultDebug(args...);
        if (log_file) {
            writeFile(msg);
            return;
        }
        std::cout << msg;
    }
    /*----------------------------------------------------------------------------------------------------*/

    /*-----------------------------------------log file functions-----------------------------------------*/
    void enableLogFile() { log_file = true; }
    void disableLogFile() { log_file = false; }
    /*----------------------------------------------------------------------------------------------------*/

    /*---------------------------------------Logging with timestamp---------------------------------------*/
    void reset() { clock.reset(); }
    uint64_t timestamp(const ResolutionLevel& level = NANO, bool since_epoch = true, bool get_decimal = false)
    {
        return clock.elapsed(level, since_epoch, get_decimal);
    }
    // start time of stopwatch
    void s() { clock.start(); }
    // stop time and return duration (unit: nanoseconds)
    uint64_t t(const ResolutionLevel& level = NANO) { return clock.stop(level); }
    /*----------------------------------------------------------------------------------------------------*/

    template<typename... Args>
    void logCSV(const Args&... args)
    {
        if (LOG_LEVEL < 1) {
            // do nothing
            return;
        }
        if (!log_file) {
            log_file = true;
            makeLogDir();
            path_to_logfolder = "log/";
        }
        // check file name
        int len = filename.length();
        if (len < 4 || filename[len - 4] != '.' || filename[len - 3] != 'c' || filename[len - 2] != 's' ||
            filename[len - 1] != 'v') {
            // create a csv file
            filename = concat("", path_to_logfolder, clock.getTime(false), ".csv");
        }
        std::string row = concat(",", args...);
        row.replace(row.end() - 1, row.end(), "\n");
        writeFile(row);
    }
};

#endif // GENERAL_LOGGER_H