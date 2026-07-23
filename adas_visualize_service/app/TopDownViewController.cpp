#include "TopDownViewController.h"

TopDownViewController::TopDownViewController(QObject *parent)
    : QObject(parent)
{
}

void TopDownViewController::updateObjects(const std::vector<AdvanceFusionObject>& adv_objs)  {
    QMutexLocker locker(&m_mutex);
    // Check Adv_Objs
    if (adv_objs.empty()) {
        return;
    }
    
    // Clear old objects
    m_objects.clear();
    
    for (const auto& obj : adv_objs) {
        m_objects.append(convertToQMLObject(obj));
    }

    emit objectsChanged();
}

void TopDownViewController::updateObjects(const wrapperAdvanceFusionObjects& wrapper)
{
    QMutexLocker locker(&m_mutex);
    // Clear old objects
    m_objects.clear();

    // Check wrapper
    if (wrapper.adv_objs.empty()) {
        emit objectsChanged();
        return;
    }
    
    
   size_t numObjects = std::min(wrapper.adv_objs.size(), wrapper.src_objs.size());
   for (size_t i = 0; i < numObjects; i++) {
        QVariantMap qmlObj;
    
        // classID 
        qmlObj["classId"] = wrapper.adv_objs[i].classId;
        qmlObj["tracking_id"] = wrapper.adv_objs[i].id_;
        qmlObj["x"] = wrapper.adv_objs[i].x_offset;
        qmlObj["y"] = wrapper.adv_objs[i].y_offset;
        qmlObj["z"] = wrapper.adv_objs[i].z_offset;

        // Velocity
        qmlObj["vx"] = wrapper.adv_objs[i].x_velocity;
        qmlObj["vy"] = wrapper.adv_objs[i].y_velocity;

        if (wrapper.src_objs[i].at(SensorType::CAMERA)) {
            qmlObj["sensor_type"] = "CAMERA";
        } else if (wrapper.src_objs[i].at(SensorType::RADAR)) {
            qmlObj["sensor_type"] = "RADAR";
        } else if (wrapper.src_objs[i].at(SensorType::FUSION)) {
            qmlObj["sensor_type"] = "FUSION";
        } else {
            qmlObj["sensor_type"] = "UNDEFINED";
        }
        m_objects.append(qmlObj);
    }

    emit objectsChanged();
}

void TopDownViewController::updateLanes(const PerceptionResults &perception_results)
{
    QMutexLocker locker(&m_mutex);
    // Constants
    static const float deltaY = 0.5;

    // Prepare holders
    QVariantList qmlLanes;

    // Convert lane markings into QML lanes
    for (LaneMarking const &laneMarking : perception_results.laneResult.laneMarkingsWorld) {
        if (laneMarking.xCoeffs.size() == 0) continue;  // Check if lane marking is valid

        // Calculate lane points
        QVariantList lanePoints;
        float yValue = std::round(laneMarking.startY * 2) / 2.0f;  // Round to 0.5 step
        while (0.0f <= yValue && yValue <= laneMarking.endY) {
            QVariantMap point;
            point["x"] = laneMarking.x(yValue);
            point["y"] = yValue;
            lanePoints.append(point);
            yValue += deltaY;
        }

        // Get lane type and combine into lane
        QVariantMap lane;
        lane["type"] = laneMarking.type;
        lane["points"] = lanePoints;
        
        qmlLanes.append(QVariant::fromValue(lane));
    }

    m_lanes = qmlLanes;
    
    emit lanesChanged();
}

void TopDownViewController::updatePlanningPath(const PlanningResults& planning_results)
{
    QMutexLocker locker(&m_mutex);
    m_planningPath = convertToQMLPlanningPath(planning_results);
    emit planningPathChanged();
}

QVariantMap TopDownViewController::convertToQMLObject(const AdvanceFusionObject& obj) {
    QVariantMap qmlObj;

    // // Handle the object in future
    // // classID 
    // qmlObj["classId"] = obj.classId;
    // qmlObj["tracking_id"] = obj.id_;
    // qmlObj["x"] = obj.x_offset;
    // qmlObj["y"] = obj.y_offset;
    // qmlObj["z"] = obj.z_offset;

    // // Velocity
    // qmlObj["vx"] = obj.x_velocity;
    // qmlObj["vy"] = obj.y_velocity;

    return qmlObj;
}

QVariantList TopDownViewController::convertToQMLPlanningPath(const PlanningResults& planning_results) {
    QVariantList qmlPathPoints;

    for (const auto& point : planning_results.points) {
        QVariantMap pathPoint;
        
        pathPoint["x"] = point.at(0);
        pathPoint["y"] = point.at(1);
        
        qmlPathPoints.append(pathPoint);
    }
    
    return qmlPathPoints;
}
