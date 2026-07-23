#include "RadarViewConfig.h"
#include <QFile>
#include <QTextStream>

RadarViewConfig::RadarViewConfig(QObject *parent) : SensorViewConfig(parent),
    m_horizontalFov(30.0),
    m_verticalFov(10.0),
    m_range(250.0)
{
}

bool RadarViewConfig::loadFromFile(const QString &filePath)
{
    // Get common sensor configuration from base class
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
        } else if (key == "range") {
            m_range = value.toDouble();
        }
    }
    
    file.close();
    emit configChanged();
    
    return success;
}