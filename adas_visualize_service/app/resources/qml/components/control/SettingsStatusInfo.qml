pragma Singleton
import QtQuick 2.14

import com.banvien.MainViewController 1.0

QtObject {
    // Level 2 properties
    property real currentSpeed: MainViewController.kmhInfo
    property real capturedSpeed: MainViewController.speedCaptured
    property real currentDistance: MainViewController.distanceInput
    property real capturedDistance: MainViewController.distanceCaptured
}