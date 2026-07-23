import QtQuick 2.14
import QtQuick.Controls 2.14

TextField {
    id: root

    property alias textFieldWidth: root.implicitWidth
    property alias textFieldHeight: root.implicitHeight
    property alias textInputSize: root.font.pixelSize
    property alias leftAnchors: root.anchors.left

    anchors.left: parent.left
    placeholderText: "192.168.240.110"
    color: "white"
    font {
        bold: false
    }

    maximumLength: 15
    validator: RegExpValidator {
        regExp: /^[0-9.]*$/
    }

    background: Rectangle {
        color: "#2B3136"
        radius: 7
    }

    renderType: Text.QtRendering
}
