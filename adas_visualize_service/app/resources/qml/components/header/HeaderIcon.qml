import QtQuick 2.14
import QtQuick.Controls 2.14

Item {
    id: headerIcon
    width: 40
    height: 40
    
    property string iconSourceOff: ""
    property string iconSourceOn: ""
    property bool isActivated: false        // Status of icon
    property bool isClickable: true
    property string iconColor: "white"

    signal clicked()
    
    Image {
        id: icon
        anchors.centerIn: parent
        source: headerIcon.isActivated ? headerIcon.iconSourceOn : headerIcon.iconSourceOff
        width: 40
        height: 40
        fillMode: Image.PreserveAspectFit
        
    }
    
    MouseArea {
        id: mouseArea
        anchors.fill: parent
        enabled: isClickable
        cursorShape: enabled ? Qt.PointingHandCursor : Qt.ArrowCursor
        onClicked: {
            headerIcon.clicked()
        }
        hoverEnabled: true
    }
}