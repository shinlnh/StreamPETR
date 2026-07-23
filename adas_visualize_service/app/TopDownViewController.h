#ifndef TOPDOWNVIEWCONTROLLER_H
#define TOPDOWNVIEWCONTROLLER_H

#include "intermediate_representations.h"

#include <QtCore/QObject>
#include <QtCore/QVariant>
#include <QtCore/QVariantList>
#include <QtCore/QString>
#include <QTimer>
#include <QDateTime>
#include <QtMath>
#include <QDebug>
#include <QMutex>
#include <vector>

struct wrapperAdvanceFusionObjects {
    std::vector<AdvanceFusionObject> adv_objs;
    std::vector<std::map<SensorType, bool>> src_objs;
};

class TopDownViewController : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVariantList objects READ objects NOTIFY objectsChanged)
    Q_PROPERTY(QVariantList lanes READ lanes NOTIFY lanesChanged)
    Q_PROPERTY(QVariantList planningPath READ planningPath NOTIFY planningPathChanged)


public:
    explicit TopDownViewController(QObject *parent = nullptr);

    QVariantList objects() const { QMutexLocker locker(&m_mutex); return m_objects; }
    QVariantList lanes() const { QMutexLocker locker(&m_mutex); return m_lanes; }
    QVariantList planningPath() const { QMutexLocker locker(&m_mutex); return m_planningPath; }
    
    Q_INVOKABLE void updateObjects(const std::vector<AdvanceFusionObject>& adv_objs);
    Q_INVOKABLE void updateObjects(const wrapperAdvanceFusionObjects& wrapper);
    Q_INVOKABLE void updateLanes(const PerceptionResults &perception_results);
    Q_INVOKABLE void updatePlanningPath(const PlanningResults& planning_results);

signals:
    void objectsChanged();
    void lanesChanged();
    void planningPathChanged();

private:
    QVariantList m_objects;
    QVariantList m_lanes;
    QVariantList m_planningPath;
    
    QVariantMap convertToQMLObject(const AdvanceFusionObject& obj);
    QVariantList convertToQMLPlanningPath(const PlanningResults& planning_results);
    
    mutable QMutex m_mutex;
};

#endif // TOPDOWNVIEWCONTROLLER_H