#ifndef AEB_DEBUG_WINDOW_H
#define AEB_DEBUG_WINDOW_H

#include <QWidget>
#include <QPainter>
#include <QMutex>
#include <ipc_helper/msg/sff_debug_data.hpp>

class AEBDebugWindow : public QWidget
{
    Q_OBJECT

public:
    explicit AEBDebugWindow(QWidget *parent = nullptr);
    ~AEBDebugWindow() override = default;

    void updateData(const ipc_helper::msg::SFFDebugData &msg);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    ipc_helper::msg::SFFDebugData cachedData_;
    QMutex dataMutex_;
    bool hasData_ = false;

    static constexpr int IMG_WIDTH = 1200;
    static constexpr int IMG_HEIGHT = 900;
    static constexpr float SCALE = 5.0f;

    QPointF worldToCanvas(float x, float y) const;

    void drawClaimSet(QPainter &painter,
                      const ipc_helper::msg::SFFClaimSet &claimSet,
                      const QColor &lineColor,
                      const QColor &circleColor,
                      const QColor &obbColor,
                      int type, int sample);

    void drawClaimState(QPainter &painter,
                        const ipc_helper::msg::SFFClaimState &cs,
                        const ipc_helper::msg::SFFClaimSet &claimSet,
                        QColor obbColor,
                        int type, int sample);

    QColor getZoneColor(float t,
                        const ipc_helper::msg::SFFClaimSet &claimSet,
                        const QColor &defaultColor,
                        int type, int sample) const;
};

#endif // AEB_DEBUG_WINDOW_H
