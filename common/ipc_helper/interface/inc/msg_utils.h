#ifndef MSG_UTILS_H
#define MSG_UTILS_H

#include "intermediate_representations.h"
#include <ipc_helper/msg/pipeline_results.hpp>

#define DEF_ASSIGN_HELPERS(type1, type2)                                   \
void operator<<(type1 &dest, type2 const &src);                            \
void operator<<(type2 &dest, type1 const &src);                            \
void operator<<(std::vector<type1> &dest, std::vector<type2> const &src);  \
void operator<<(std::vector<type2> &dest, std::vector<type1> const &src);

#define MAKE_VECTOR_ASSIGN_HELPER_OF(type1, type2)                            \
void operator<<(std::vector<type1> &dest, std::vector<type2> const &src)      \
{                                                                             \
    size_t n = src.size();                                                    \
    dest.resize(n);                                                           \
    for (size_t i = 0; i < n; i++) dest[i] << src[i];                         \
}

#define MAKE_VECTOR_ASSIGN_HELPERS(type1, type2)  \
MAKE_VECTOR_ASSIGN_HELPER_OF(type1, type2)        \
MAKE_VECTOR_ASSIGN_HELPER_OF(type2, type1)

// Level: Low
DEF_ASSIGN_HELPERS(Box, ipc_helper::msg::Box)
DEF_ASSIGN_HELPERS(cv::Point2f, ipc_helper::msg::Point2f)

// Level: Medium
DEF_ASSIGN_HELPERS(AdvanceFusionObject, ipc_helper::msg::AdvanceFusionObject)
DEF_ASSIGN_HELPERS(LanePointsRaw, ipc_helper::msg::LanePointsRaw)
DEF_ASSIGN_HELPERS(LaneMarking, ipc_helper::msg::LaneMarking)

// Level: High
DEF_ASSIGN_HELPERS(LaneDetection, ipc_helper::msg::LaneDetection)


#endif // MSG_UTILS_H