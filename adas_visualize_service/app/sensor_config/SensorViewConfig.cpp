#include "SensorViewConfig.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>

SensorViewConfig::SensorViewConfig(QObject *parent) : QObject(parent)
{
    // Initialize coordinate with default values [x, y, z, roll, pitch, yaw]
    m_coordinate << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0;
}

bool SensorViewConfig::loadFromFile(const QString &filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Could not open file:" << filePath;
        return false;
    }
    
    QTextStream in(&file);
    bool success = true;
    
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        
        // Skip empty lines
        if (line.isEmpty()) continue;
        
        // Split by colon to get key-value pairs
        QStringList parts = line.split(":", QString::SkipEmptyParts);
        if (parts.size() != 2) {
            qWarning() << "Invalid line format:" << line;
            continue;
        }
        
        QString key = parts[0].trimmed();
        QString value = parts[1].trimmed();
        
        if (key == "type") {
            m_type = value;
        } else if (key == "id") {
            m_id = value;
        } else if (key == "coordinate") {
            // Parse the coordinates (format: x, y, z, roll, pitch, yaw)
            m_coordinate.clear();
            QStringList coords = value.split(",", QString::SkipEmptyParts);
            for (const QString &coord : coords) {
                bool ok;
                double val = coord.trimmed().toDouble(&ok);
                if (ok) {
                    m_coordinate.append(val);
                } else {
                    qWarning() << "Invalid coordinate value:" << coord;
                    success = false;
                }
            }
            
            // Ensure we have all 6 values (x, y, z, roll, pitch, yaw)
            while (m_coordinate.size() < 6) {
                m_coordinate.append(0.0);
            }
        }
    }
    
    file.close();
    emit configChanged();
    
    return success;
}