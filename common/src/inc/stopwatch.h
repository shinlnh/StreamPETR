#ifndef STOPWATCH_H
#define STOPWATCH_H

#include <chrono>
#include <ctime>
#include <string>
#include <sstream>

enum ResolutionLevel {
    NANO = 0,
    MICRO = 1,
    MILLI = 2,
    SEC = 3
};

class Stopwatch {
private:
    typedef std::chrono::high_resolution_clock::time_point tp;
    typedef std::chrono::high_resolution_clock clock;
    typedef std::chrono::milliseconds ms;
    typedef std::chrono::microseconds us;
    typedef std::chrono::nanoseconds ns;
    typedef std::chrono::seconds sec;
    tp base, begin;

protected: 
    uint64_t getDuration(const tp &tp_from, const tp &tp_to, const ResolutionLevel& level) {
        switch (level) {
        case NANO:
            return std::chrono::duration_cast<ns>(tp_to - tp_from).count();
        case MICRO:
            return std::chrono::duration_cast<us>(tp_to - tp_from).count();
        case MILLI:
            return std::chrono::duration_cast<ms>(tp_to - tp_from).count();
        case SEC:
            return std::chrono::duration_cast<sec>(tp_to - tp_from).count();
        }    
        return -1;
    }
public:
    Stopwatch() : base(clock::now()), begin(clock::now()) {}
    ~Stopwatch() {}
    void reset() { base = clock::now(); }
    uint64_t elapsed(const ResolutionLevel& level = NANO, bool since_epoch = true, bool get_decimal = false) {
        if (since_epoch) {
            switch (level) {
            case NANO:
                return std::chrono::duration_cast<ns>(clock::now().time_since_epoch()).count(); 
            case MICRO:
                return std::chrono::duration_cast<us>(clock::now().time_since_epoch()).count(); 
            case MILLI:
                return std::chrono::duration_cast<ms>(clock::now().time_since_epoch()).count(); 
            case SEC:
                return std::chrono::duration_cast<sec>(clock::now().time_since_epoch()).count(); 
            default:
                break;
            }
        } else {
            if (get_decimal) {
                double duration = getDuration(base, clock::now(), NANO) / 1.0;
                for (int i = level; i > 0; --i) {
                    duration /= 1000.0;
                }
                return static_cast<uint64_t>(duration);
            }
            return getDuration(base, clock::now(), level);
        }
        return -1;
    }

    // get current time with human readable format 
    std::string getTime(bool untilSec = true) {
        // format: <year>_<month>_<day>_<hr>-<min>-<sec>
        // make sure format: yyyy_mm_dd_hh-mm-ss
        tp now = clock::now();
        std::time_t cvt_tt = clock::to_time_t(now);
        tm *local_time = localtime(&cvt_tt);
        int year = local_time->tm_year + 1900,  // base in year 1900
            month = local_time->tm_mon + 1,     // base month is 0
            day = local_time->tm_mday, 
            hour = local_time->tm_hour, 
            min = local_time->tm_min, 
            sec = local_time->tm_sec;

        std::ostringstream result;
        result << year << '_';
        if (month < 10) result << '0';
        result << month << '_';
        if (day < 10) result << '0';
        result << day;

        if (untilSec) {
            result << '_';
            if (hour < 10) result << '0';
            result << hour << '-';
            if (min < 10) result << '0';
            result << min << '-';
            if (sec < 10) result << '0';
            result << sec;
        }
        return result.str();
    }

    void start() { begin = clock::now(); }
    uint64_t stop(const ResolutionLevel& level = NANO) { return getDuration(begin, clock::now(), level); }
};

#endif  // STOPWATCH_H
