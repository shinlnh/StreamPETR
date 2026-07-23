#ifndef COMMON_H
#define COMMON_H
/*include C++*/
#include <iostream>
#include <functional>

#ifndef LOG_LEVEL
#define LOG_LEVEL 3
#endif

#define LOG_LEVEL_NONE     0
#define LOG_LEVEL_ERROR    1
#define LOG_LEVEL_WARN     2
#define LOG_LEVEL_INFO     3
#define LOG_LEVEL_DEBUG    4

#define ERROR( ... )
#define WARN( ... )
#define INFO( ... )
#define DEBUG( ... )

#if LOG_LEVEL >= LOG_LEVEL_ERROR
#undef  ERROR
#define ERROR(format, ...) printf("[ERROR][%s:%d]: " format "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#endif
#if LOG_LEVEL >= LOG_LEVEL_WARN
#undef  WARN
#define WARN(format, ...) printf("[WARN][%s:%d]: " format "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#endif
#if LOG_LEVEL >= LOG_LEVEL_INFO
#undef  INFO
#define INFO(format, ...) printf("[INFO][%s:%d]: " format "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#endif
#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#undef  DEBUG
#define DEBUG(format, ...) printf("[DEBUG][%s:%d]: " format "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#endif

/*return function status*/
typedef enum e_bv_err
{
    BV_RETURN_OK = 0,     /*!< correct function execution */
    BV_RETURN_ERROR = -1, /*!< function return error */
} bv_err_return_t;

/*Using to get address of std::function*/
template <typename T, typename... U>
size_t getAddress(std::function<T(U...)> f)
{
    typedef T(fnType)(U...);
    fnType **fnPointer = f.template target<fnType *>();
    return (size_t)*fnPointer;
}

/**
 * @brief Clamp function to limit the value within a boundary.
 */
template <typename T>
static inline T clamp(T value, T lowerLimit, T higherLimit) {
    if (lowerLimit > higherLimit)
        std::swap(lowerLimit, higherLimit);
    if (value < lowerLimit)
        return lowerLimit;
    if (value > higherLimit)
        return higherLimit;
    return value;
}
#endif
