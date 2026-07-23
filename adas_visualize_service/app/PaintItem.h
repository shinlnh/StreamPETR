#ifndef __PAINTITEM_H__
#define __PAINTITEM_H__

#include <QObject>
#include <QtQuick>
#include <QImage>
#include <QString>

#define PADDING_LEFT_RIGHT 8
#define DEFAULT_SPACING 8
#define PADDING_TOP_BOTTOM 5
#define FONT "Arial"
#define FONT_SIZE 13

class PaintItem : public QQuickPaintedItem
{
    Q_OBJECT
public:
    PaintItem(QQuickItem *parent = nullptr);
    void paint(QPainter *painter) override;
    void paintVisualize_MainScreen(QPainter *painter);
    ~PaintItem();

public slots:
    void updateImage(const QImage& image);
    void setText(const QString& text);  
    void setstate(bool state);

protected:
    QImage *imageToSend;
    QString textToDisplay;  
    bool m_state = false; 
};

#endif // __PAINTITEM_H__
