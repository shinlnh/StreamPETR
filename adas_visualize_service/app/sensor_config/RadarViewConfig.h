#ifndef RADARVIEWCONFIG_H
#define RADARVIEWCONFIG_H

#include "SensorViewConfig.h"

class RadarViewConfig : public SensorViewConfig
{
    Q_OBJECT
    Q_PROPERTY(qreal horizontalFov READ horizontalFov NOTIFY configChanged)
    Q_PROPERTY(qreal verticalFov READ verticalFov NOTIFY configChanged)
    Q_PROPERTY(qreal range READ range NOTIFY configChanged)

public:
    explicit RadarViewConfig(QObject *parent = nullptr);
    
    bool loadFromFile(const QString &filePath) override;
    
    // Radar-specific getters
    qreal horizontalFov() const { return m_horizontalFov; }
    qreal verticalFov() const { return m_verticalFov; }
    qreal range() const { return m_range; }
    
private:
    qreal m_horizontalFov;
    qreal m_verticalFov;
    qreal m_range;
};

#endif // RADARVIEWCONFIG_H