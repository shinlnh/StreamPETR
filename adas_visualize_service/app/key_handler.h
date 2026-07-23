#ifndef KEY_HANDLER_H
#define KEY_HANDLER_H

#include "MainViewController.h"
#include "adas_main_visualize.h"

#include <QObject>
#include <QKeyEvent>
#include <QTimer>
#include <QSet>

class KeyPressHandler : public QObject {
    Q_OBJECT
public:
    KeyPressHandler(QObject *parent = nullptr, MainViewController *controller = nullptr, AdasMainVisualize *adas_main_visualize_instace = nullptr);

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;

signals:
    void keysChanged(const QSet<int> &keys);

private:
    AdasMainVisualize* adas_main_visualize_ptr;
    MainViewController *adas_ptr;

    QSet<int> pressedKeys;
    float m_thottleSlider = 0.0;
    float m_brakeSlider = 0.0;
    float m_steerSlider = 0.0;
    bool reset = false;
    QTimer timer;

    void adjustDirection();
    void printDirectionState();
    void vehicleControlUnit();
};

#endif // KEY_HANDLER_H
