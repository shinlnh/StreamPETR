#include "msg_utils.h"


/******************************** LEVEL: LOW *********************************/
// -------- Box ------------------------------------------------------------ //
/** @brief Helper functions to copy Box */
void operator<<(Box &dest, ipc_helper::msg::Box const &src)
{
    dest.x1 = src.x1;
    dest.y1 = src.y1;
    dest.x2 = src.x2;
    dest.y2 = src.y2;
}

/** @brief Helper functions to copy Box */
void operator<<(ipc_helper::msg::Box &dest, Box const &src)
{
    dest.x1 = src.x1;
    dest.y1 = src.y1;
    dest.x2 = src.x2;
    dest.y2 = src.y2;
}

MAKE_VECTOR_ASSIGN_HELPERS(Box, ipc_helper::msg::Box)


// -------- Point2f -------------------------------------------------------- //
/** @brief Helper functions to copy Point2f */
void operator<<(cv::Point2f &dest, ipc_helper::msg::Point2f const &src)
{
    dest.x = src.x;
    dest.y = src.y;
}

/** @brief Helper functions to copy Point2f */
void operator<<(ipc_helper::msg::Point2f &dest, cv::Point2f const &src)
{
    dest.x = src.x;
    dest.y = src.y;
}

MAKE_VECTOR_ASSIGN_HELPERS(cv::Point2f, ipc_helper::msg::Point2f)


/******************************** LEVEL: MEDIUM ******************************/
// -------- AdvanceFusionObject -------------------------------------------- //
/** @brief Helper functions to copy AdvanceFusionObject */
void operator<<(AdvanceFusionObject &dest, ipc_helper::msg::AdvanceFusionObject const &src)
{
    dest.bbox << src.bbox;
    dest.x_offset = src.x_offset;
    dest.y_offset = src.y_offset;
    dest.x_velocity = src.x_velocity;
    dest.y_velocity = src.y_velocity;
    dest.classId = src.class_id;
    dest.id_ = src.id;
}

/** @brief Helper functions to copy AdvanceFusionObject */
void operator<<(ipc_helper::msg::AdvanceFusionObject &dest, AdvanceFusionObject const &src)
{
    dest.bbox << src.bbox;
    dest.x_offset = src.x_offset;
    dest.y_offset = src.y_offset;
    dest.x_velocity = src.x_velocity;
    dest.y_velocity = src.y_velocity;
    dest.class_id = src.classId;
    dest.id = src.id_;
}

MAKE_VECTOR_ASSIGN_HELPERS(AdvanceFusionObject, ipc_helper::msg::AdvanceFusionObject)


// -------- LanePointsRaw -------------------------------------------------- //
/** @brief Helper functions to copy LanePointsRaw */
void operator<<(LanePointsRaw &dest, ipc_helper::msg::LanePointsRaw const &src)
{
    dest.type = src.type;
    dest.points << src.points;
}

/** @brief Helper functions to copy LanePointsRaw */
void operator<<(ipc_helper::msg::LanePointsRaw &dest, LanePointsRaw const &src)
{
    dest.type = src.type;
    dest.points << src.points;
}

MAKE_VECTOR_ASSIGN_HELPERS(LanePointsRaw, ipc_helper::msg::LanePointsRaw)


// -------- LaneMarking ---------------------------------------------------- //
/** @brief Helper functions to copy LaneMarking */
void operator<<(LaneMarking &dest, ipc_helper::msg::LaneMarking const &src)
{
    dest.type = src.type;
    dest.startY = src.start_y;
    dest.endY = src.end_y;
    dest.xCoeffs = src.x_coeffs;
    // dest.zCoeffs = src.z_coeffs;
}

/** @brief Helper functions to copy LaneMarking */
void operator<<(ipc_helper::msg::LaneMarking &dest, LaneMarking const &src)
{
    dest.type = src.type;
    dest.start_y = src.startY;
    dest.end_y = src.endY;
    dest.x_coeffs = src.xCoeffs;
    // dest.z_coeffs = src.zCoeffs;
}

MAKE_VECTOR_ASSIGN_HELPERS(LaneMarking, ipc_helper::msg::LaneMarking)


/******************************** LEVEL: HIGH ********************************/
// -------- LaneDetection -------------------------------------------------- //
/** @brief Helper functions to copy LaneDetection */
void operator<<(LaneDetection &dest, ipc_helper::msg::LaneDetection const &src)
{
    dest.laneMarkingsPoint << src.lane_markings_point;  // Copy raw points
    dest.laneMarkingsWorld << src.lane_markings_world;  // Copy world lane markings
}

/** @brief Helper functions to copy LaneDetection */
void operator<<(ipc_helper::msg::LaneDetection &dest, LaneDetection const &src)
{
    dest.lane_markings_point << src.laneMarkingsPoint;  // Copy raw points
    dest.lane_markings_world << src.laneMarkingsWorld;  // Copy world lane markings
}

MAKE_VECTOR_ASSIGN_HELPERS(LaneDetection, ipc_helper::msg::LaneDetection)