import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

Rectangle {
    id: root
    
    property bool layoutFillWidth: false
    property int layoutPreferredHeight: height
    
    Layout.fillWidth: layoutFillWidth
    Layout.preferredHeight: layoutPreferredHeight
    
    radius: 15
    
    gradient: Gradient {
        GradientStop { position: 0.0; color: Qt.rgba(13/255, 17/255, 26/255, 1.0) }  // #0D111A
        GradientStop { position: 1.0; color: Qt.rgba(21/255, 25/255, 33/255, 0.55) }   // #151921 
        orientation: Gradient.Vertical
    }
}