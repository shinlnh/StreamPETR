#include "CameraViewConfig.h"
#include <QFile>
#include <QTextStream>

CameraViewConfig::CameraViewConfig(QObject *parent) : SensorViewConfig(parent),
    m_horizontalFov(30.0),
    m_verticalFov(10.0),
    m_imageSizeX(800),
    m_imageSizeY(600)
{
}

bool CameraViewConfig::loadFromFile(const QString &filePath)
{
    bool success = SensorViewConfig::loadFromFile(filePath);
    
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return false;
    }
    
    QTextStream in(&file);
    
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        
        // Skip empty lines
        if (line.isEmpty()) continue;
        
        // Split by colon to get key-value pairs
        QStringList parts = line.split(":", QString::SkipEmptyParts);
        if (parts.size() != 2) continue;
        
        QString key = parts[0].trimmed();
        QString value = parts[1].trimmed();
        
        // Process key-value pairs
        if (key == "horizontal_fov") {
            m_horizontalFov = value.toDouble();
        } else if (key == "vertical_fov") {
            m_verticalFov = value.toDouble();
        } else if (key == "image_size_x") {
            m_imageSizeX = value.toInt();
        } else if (key == "image_size_y") {
            m_imageSizeY = value.toInt();
        }
    }
    
    file.close();
    emit configChanged();
    
    return success;
}