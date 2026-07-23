import QtQuick 2.14
import QtQuick.Layouts 1.3

Rectangle {
    id: root

    signal clicked

    property alias textValue: textField.text
    property alias imageSource: imageField.source
    property alias colorButton: root.color
    property bool checked: false

    width: 40
    height: 40

    border.color: "#111111"
    border.width: 2
    radius: width / 2

    ColumnLayout {
        id: imageTextLayout

        anchors.centerIn: parent

        spacing: 0

        Image {
            id: imageField

            Layout.alignment: Qt.AlignCenter

            width: 32
            height: 32

            smooth: true
        }

        Text {
            id: textField

            color: checked ? "#D38D00" : "#A1A1A1"

            font {
                family: "Liberation Mono"
                pixelSize: 10
                bold: false
            }
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
