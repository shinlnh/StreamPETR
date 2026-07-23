#include "PaintItem.h"
#include <vector>
PaintItem::PaintItem(QQuickItem *parent)
    : QQuickPaintedItem(parent), imageToSend(new QImage(200, 200, QImage::Format_RGB32))
{
    imageToSend->fill(QColor(78, 78, 78));
}

void PaintItem::paint(QPainter *painter)
{
    QPainterPath roundedRect;
    roundedRect.addRoundedRect(boundingRect(), 15, 15);

    painter->setClipPath(roundedRect);

    painter->drawImage(boundingRect(), *imageToSend);
    if (m_state && imageToSend->width() != 0 && imageToSend->height() != 0) 
    {
        paintVisualize_MainScreen(painter);
    }
}

PaintItem::~PaintItem()
{
    delete imageToSend;
}

void PaintItem::updateImage(const QImage& image)
{
    *imageToSend = image;
    update();
}

void PaintItem::setText(const QString& text)
{
    textToDisplay = text;
    update();
}

void PaintItem::setstate(bool state)
{
    m_state = state;
    update();
}

void PaintItem::paintVisualize_MainScreen(QPainter *painter)
{

    painter->setPen(Qt::white);
    painter->setFont(QFont("FONT", FONT_SIZE));

    QStringList lines = textToDisplay.split(';');
    int x_Offset = 0;
    int y_Offset = 0;
    const int spacing = DEFAULT_SPACING;  

    for (int i = 0; i < lines.size(); ++i)
    {
        QString trimmedLine = lines[i].trimmed();
        if (trimmedLine.isEmpty()) {
            continue; 
        }

        QRectF textRectF = painter->boundingRect(QRectF(), Qt::TextSingleLine, trimmedLine);

        // Balance width text and width image
        if (x_Offset + textRectF.width() > imageToSend->width() + 2.5*DEFAULT_SPACING ) {
            x_Offset = 0;
            y_Offset += textRectF.height() + 2 * PADDING_TOP_BOTTOM + spacing;
        }

        QRect textRect = QRect(x_Offset + DEFAULT_SPACING,
                               y_Offset + DEFAULT_SPACING,
                               static_cast<int>(textRectF.width()) + 2 * PADDING_LEFT_RIGHT,
                               static_cast<int>(textRectF.height()) + 2 * PADDING_TOP_BOTTOM);

        // Round concern of background
        QPainterPath textBackgroundPath;
        textBackgroundPath.addRoundedRect(textRect, 5, 5);

        // Fill background color
        painter->fillPath(textBackgroundPath, QColor(0, 0, 0, 128));

        // Ajust Padding of Text
        painter->drawText(textRect.adjusted(PADDING_LEFT_RIGHT, PADDING_TOP_BOTTOM, -PADDING_LEFT_RIGHT, -PADDING_TOP_BOTTOM), Qt::TextSingleLine, trimmedLine);

        // Set x_Offset
        x_Offset += textRectF.width() + 2 * PADDING_LEFT_RIGHT + spacing;
    }
}
