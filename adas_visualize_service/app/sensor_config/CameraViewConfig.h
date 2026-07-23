#ifndef CAMERAVIEWCONFIG_H
#define CAMERAVIEWCONFIG_H

#include "SensorViewConfig.h"

class CameraViewConfig : public SensorViewConfig
{
    Q_OBJECT
    Q_PROPERTY(qreal horizontalFov READ horizontalFov NOTIFY configChanged)
    Q_PROPERTY(qreal verticalFov READ verticalFov NOTIFY configChanged)
    Q_PROPERTY(int imageSizeX READ imageSizeX NOTIFY configChanged)
    Q_PROPERTY(int imageSizeY READ imageSizeY NOTIFY configChanged)

public:
    explicit CameraViewConfig(QObject *parent = nullptr);
    
    bool loadFromFile(const QString &filePath) override;
    
    // Camera-specific getters
    qreal horizontalFov() const { return m_horizontalFov; }
    qreal verticalFov() const { return m_verticalFov; }
    int imageSizeX() const { return m_imageSizeX; }
    int imageSizeY() const { return m_imageSizeY; }
    
private:
    qreal m_horizontalFov;
    qreal m_verticalFov;
    int m_imageSizeX;
    int m_imageSizeY;
};

#endif // CAMERAVIEWCONFIG_H