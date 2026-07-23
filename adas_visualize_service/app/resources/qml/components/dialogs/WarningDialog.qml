import QtQuick 2.14
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14

Dialog {
    id: warningDialog
    
    property string dialogTitle: "WARNING"
    property string dialogMessage: "Turn off <>"
    
    title: dialogTitle
    width: 307
    height: 188
    modal: true
    focus: true
    closePolicy: Dialog.CloseOnEscape | Dialog.CloseOnPressOutside
    
    // Center the dialog in the window
    // x: (ApplicationWindow.window ? (ApplicationWindow.window.width - width) / 2 : 0)
    // y: (ApplicationWindow.window ? (ApplicationWindow.window.height - height) / 2 : 0)
    
    // Fine tune the position
    x: 1080
    y: 700
    
    // Disable the modal background
    Overlay.modal: Rectangle {
        color: "transparent"
    }
    
    header: null
    footer: null
    
    padding: 0
    
    // Main dialog container with visible border
    background: Rectangle {
        id: mainBackground
        color: "#FFF4DE"
        // border.width: 2
        // border.color: "#6A3C1E"
        radius: 15
    }
    
    // Dialog content
    contentItem: Item {
        anchors.fill: parent
        
        // Header area
        Rectangle {
            id: headerRect
            anchors {
                top: parent.top
                left: parent.left
                right: parent.right
            }
            height: 50
            color: "#FCD26D"
            radius: 15
            
            // Header
            Rectangle {
                anchors {
                    left: parent.left
                    right: parent.right
                    bottom: parent.bottom
                }
                height: parent.height / 2
                color: parent.color
            }
            
            // Title
            Text {
                anchors {
                    left: parent.left
                    verticalCenter: parent.verticalCenter
                    leftMargin: 16
                }
                text: warningDialog.title
                color: "#57360B"
                font {
                    pixelSize: 24
                    bold: true
                    family: "Liberation Sans, Sans-serif"
                }
            }
        }
        
        // Content
        Item {
            id: contentArea
            anchors {
                top: headerRect.bottom
                left: parent.left
                right: parent.right
                bottom: parent.bottom
            }
            
            Text {
                id: messageText
                anchors {
                    left: parent.left
                    top: parent.top
                    leftMargin: 16
                    topMargin: 16
                }
                text: warningDialog.dialogMessage
                color: "#350100"
                font {
                    pixelSize: 16
                    bold: false
                    family: "Liberation Sans, Sans-serif"
                }
            }
            
            // Confirm button
            Button {
                id: okButton
                anchors {
                    right: parent.right
                    bottom: parent.bottom
                    bottomMargin: 16
                    rightMargin: 16
                }
                width: 80
                height: 30
                
                contentItem: Text {
                    text: "OK"
                    color: "#6A3C1E"
                    font {
                        pixelSize: 14
                        bold: true
                        family: "Liberation Sans, Sans-serif"
                    }
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                
                background: Rectangle {
                    radius: 12
                    color: "#FCD26D"
                    border.width: 2
                    border.color: "#6A3C1E"
                }
                
                onClicked: {
                    warningDialog.close()
                }
            }
        }
        
        // Draw a border overlay (Border round the dialog)
        Rectangle {
            id: borderOverlay
            anchors.fill: parent
            anchors.margins: -1
            color: "transparent"
            radius: 15
            border.width: 3
            border.color: "#6A3C1E"
            z: 1
        }
    }
}