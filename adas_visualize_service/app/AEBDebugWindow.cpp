#include "AEBDebugWindow.h"
#include <QPaintEvent>
#include <cmath>

AEBDebugWindow::AEBDebugWindow(QWidget *parent)
    : QWidget(parent)
{
    setWindowTitle("AEB Debug - ClaimSet Top-Down");
    setFixedSize(IMG_WIDTH, IMG_HEIGHT);
    setAttribute(Qt::WA_OpaquePaintEvent);
}

void AEBDebugWindow::updateData(const ipc_helper::msg::SFFDebugData &msg)
{
    {
        QMutexLocker lock(&dataMutex_);
        cachedData_ = msg;
        hasData_ = true;
    }
    // Schedule repaint on the GUI thread
    QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
}

QPointF AEBDebugWindow::worldToCanvas(float x, float y) const
{
    float ox = IMG_WIDTH / 2.0f;
    float oy = IMG_HEIGHT - 20.0f;
    return QPointF(ox + x * SCALE, oy - y * SCALE);
}

QColor AEBDebugWindow::getZoneColor(float t,
                                     const ipc_helper::msg::SFFClaimSet &cs,
                                     const QColor &defaultColor,
                                     int type, int sample) const
{
    if (type != 1 || (sample != 0 && sample != 1 && sample != 3))
        return defaultColor;

    if (t <= cs.time_danger + 0.005f) {
        return QColor(255, 0, 0); // red
    }
    if (t <= cs.time_danger + cs.time_reaction + 0.005f && !cs.in_low_vel_range) {
        if (cs.time_reaction != 0)
            return QColor(200, 200, 200); // gray
        else
            return QColor(255, 0, 0); // red
    }
    if (t <= cs.time_danger + cs.time_warning + 0.005f) {
        return QColor(255, 255, 0); // yellow
    }

    if (sample == 3 && t == cs.time_hit) {
        return QColor(125, 220, 125); // green overlap
    }

    return defaultColor;
}

void AEBDebugWindow::drawClaimState(QPainter &painter,
                                     const ipc_helper::msg::SFFClaimState &cs,
                                     const ipc_helper::msg::SFFClaimSet &claimSet,
                                     QColor obbColor,
                                     int type, int sample)
{
    if (cs.shape_type == 0) { // CIRCLE
        QPointF center = worldToCanvas(cs.circle_center_x, cs.circle_center_y);
        float radius = cs.circle_radius * SCALE;
        if (std::isfinite(center.x()) && std::isfinite(center.y()) && std::isfinite(radius)) {
            QColor color = getZoneColor(cs.t, claimSet, obbColor, type, sample);
            painter.setPen(QPen(color, 2));
            painter.setBrush(Qt::NoBrush);
            painter.drawEllipse(center, radius, radius);
        }
    } else { // OBB
        QColor color = getZoneColor(cs.t, claimSet, obbColor, type, sample);

        QPointF center = worldToCanvas(cs.obb_center_x, cs.obb_center_y);
        float yaw_canvas = -cs.obb_yaw;
        float cos_y = std::cos(yaw_canvas);
        float sin_y = std::sin(yaw_canvas);
        QPointF dir_long(cos_y, sin_y);
        QPointF dir_lat(-sin_y, cos_y);

        QPointF hl(dir_long.x() * cs.obb_half_l * SCALE,
                   dir_long.y() * cs.obb_half_l * SCALE);
        QPointF hw(dir_lat.x() * cs.obb_half_w * SCALE,
                   dir_lat.y() * cs.obb_half_w * SCALE);

        QPolygonF poly;
        poly << (center + hl + hw)
             << (center + hl - hw)
             << (center - hl - hw)
             << (center - hl + hw);

        bool valid = true;
        for (const auto &pt : poly) {
            if (!std::isfinite(pt.x()) || !std::isfinite(pt.y())) {
                valid = false;
                break;
            }
        }
        if (!valid) return;

        // Darken fill color for sample==0 (ego main trajectory)
        if (type == 1 && sample == 0) {
            if (color == QColor(255, 0, 0))
                color = QColor(125, 0, 0);
            else if (color == QColor(200, 200, 200))
                color = QColor(125, 125, 125);
            else if (color == QColor(255, 255, 0))
                color = QColor(125, 125, 0);
        }

        painter.setPen(QPen(color, 1));
        painter.setBrush(Qt::NoBrush);
        painter.drawPolygon(poly);
    }
}

void AEBDebugWindow::drawClaimSet(QPainter &painter,
                                   const ipc_helper::msg::SFFClaimSet &claimSet,
                                   const QColor &lineColor,
                                   const QColor &circleColor,
                                   const QColor &obbColor,
                                   int type, int sample)
{
    const auto &states = claimSet.claim_states;
    if (states.empty()) return;

    // Draw path polylines
    for (size_t i = 0; i + 1 < states.size(); ++i) {
        QPointF p1 = worldToCanvas(states[i].pos_x, states[i].pos_y);
        QPointF p2 = worldToCanvas(states[i + 1].pos_x, states[i + 1].pos_y);

        if (!std::isfinite(p1.x()) || !std::isfinite(p1.y()) ||
            !std::isfinite(p2.x()) || !std::isfinite(p2.y()))
            continue;

        QColor lc = (type == 1 && (sample == 0 || sample == 1))
                     ? getZoneColor(states[i].t, claimSet, lineColor, type, sample)
                     : lineColor;

        painter.setPen(QPen(lc, 1));
        painter.drawLine(p1, p2);
    }

    // Draw shapes at each sampled state
    for (size_t i = 0; i < states.size(); ++i) {
        drawClaimState(painter, states[i], claimSet, obbColor, type, sample);
    }
}

void AEBDebugWindow::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, false);

    // Black background
    painter.fillRect(rect(), Qt::black);

    QMutexLocker lock(&dataMutex_);
    if (!hasData_) return;

    const auto &data = cachedData_;

    // Draw ego sets
    for (int i = 0; i < (int)data.ego_sets.size(); ++i) {
        if (!data.ego_sets[i].claim_states.empty()) {
            drawClaimSet(painter, data.ego_sets[i],
                         QColor(255, 125, 230),  // lineColor (BGR 230,125,255 -> RGB)
                         QColor(0, 255, 0),      // circleColor
                         QColor(128, 128, 255),  // obbColor (BGR 255,128,128 -> RGB)
                         1, i);
        }
    }

    // Draw object sets
    for (int i = 0; i < (int)data.obj_sets.size(); ++i) {
        if (!data.obj_sets[i].claim_states.empty()) {
            drawClaimSet(painter, data.obj_sets[i],
                         QColor(255, 255, 0),    // lineColor (BGR 0,255,255 -> RGB)
                         QColor(255, 128, 0),    // circleColor (BGR 0,128,255 -> RGB)
                         QColor(128, 0, 128),    // obbColor
                         2, i);
        }
    }

    // Draw overlap points if WARNING or DANGER
    int decision = data.decision;
    if (decision == 1 || decision == 2) { // WARNING or DANGER
        const auto &overlap = data.overlap;

        // Ego overlap
        ipc_helper::msg::SFFClaimSet ego_overlap_cs;
        ego_overlap_cs.claim_states.push_back(overlap.ego_overlap);
        ego_overlap_cs.in_stop_state = overlap.ego_in_stop_state;
        ego_overlap_cs.time_hit = overlap.t_hit_ego;
        ego_overlap_cs.time_danger = 0;
        ego_overlap_cs.time_reaction = 0;
        ego_overlap_cs.time_warning = 0;

        QColor overlapColor(125, 220, 125); // green
        drawClaimSet(painter, ego_overlap_cs,
                     QColor(255, 255, 0), overlapColor, overlapColor, 1, 3);

        // Object overlap
        ipc_helper::msg::SFFClaimSet obj_overlap_cs;
        obj_overlap_cs.claim_states.push_back(overlap.obj_overlap);
        obj_overlap_cs.time_hit = overlap.t_hit_obj;
        obj_overlap_cs.time_danger = 0;
        obj_overlap_cs.time_reaction = 0;
        obj_overlap_cs.time_warning = 0;

        drawClaimSet(painter, obj_overlap_cs,
                     QColor(255, 255, 0), overlapColor, overlapColor, 1, 3);
    }
}
