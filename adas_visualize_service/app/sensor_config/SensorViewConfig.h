#ifndef SENSORVIEWCONFIG_H
#define SENSORVIEWCONFIG_H

#include <QObject>
#include <QVariantList>
#include <QString>

class SensorViewConfig : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString type READ type NOTIFY configChanged)
    Q_PROPERTY(QString id READ id NOTIFY configChanged)
    Q_PROPERTY(QVariantList coordinate READ coordinate NOTIFY configChanged)

public:
    explicit SensorViewConfig(QObject *parent = nullptr);
    virtual ~SensorViewConfig() = default;
    
    virtual bool loadFromFile(const QString &filePath);
    
    QString type() const { return m_type; }
    QString id() const { return m_id; }
    QVariantList coordinate() const { return m_coordinate; }
    
signals:
    void configChanged();
    
protected:
    QString m_type;
    QString m_id;
    QVariantList m_coordinate;
};

#endif // SENSORVIEWCONFIG_H