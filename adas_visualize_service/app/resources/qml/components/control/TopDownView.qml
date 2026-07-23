import QtQuick 2.14

Item {
    id: topDownView

    property var objects: topDownViewController.objects
    property var pathPoints: topDownViewController.planningPath
    property var lanes: topDownViewController.lanes
    
    property real realCarLength: 4.79  // metter
    property real realCarWidth: 2.16   // metter
    
    // LidarCircle
    GirdRound {
        id: gridRoundView
        anchors.fill: parent
    }

    FOV {
        id: fovView
        anchors.fill: parent
        showRadar: true
        showCamera: true
        z: 1
    }
    
    // Lanes
    Lanes {
        id: lanesView
        anchors.fill: parent
        lanes: parent.lanes
    }
    
    // Path Planning
    PathPlanning {
        id: pathPlanningView
        anchors.fill: parent
        pathPoints: parent.pathPoints
    }

    // Objects Around
    Objects {
        id: objectsAround
        anchors.fill: parent
        objects: parent.objects
    }

    // Ego Vehicle
    Image {
        id: ego_vehicle
        
        width: fovView.meterToPixelRatio * realCarWidth
        height: fovView.meterToPixelRatio * realCarLength
        
        source: "qrc:/assets/model3.png"
        anchors {
            horizontalCenter: parent.horizontalCenter
            bottom: parent.bottom
            bottomMargin: objectsAround.verticalOffset - height + 2.2 * fovView.meterToPixelRatio
        }
        
        onWidthChanged: {}
        onHeightChanged: {}
    }
}