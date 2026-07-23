import QtQuick 2.14

Rectangle {
    id: root

    signal clicked

    property alias textValue: textField.text
    property alias colorButton: root.color
    property bool checked: false
    property alias borderColor: root.border.color
    property alias borderWidth: root.border.width
    property alias buttonRadius: root.radius

    border.color: "black"
    radius: 10

    Text {
        id: textField

        anchors.centerIn: parent
        color: "white"

        font {
            pixelSize: 16
            bold: false
            family: "Liberation Mono"
        }
    }

    opacity: buttonArea.containsMouse ? 0.75 : 1
    layer.mipmap: true

    MouseArea {
        id: buttonArea

        anchors.fill: parent
        hoverEnabled: true

        onClicked: {
            root.clicked()
        }
    }
}
