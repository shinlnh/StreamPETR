import QtQuick 2.14

Image {
    id: root
    
    property string buttonType: "plus"
    property var onButtonClicked: function() {}
    property bool enabled: true
    property real scaleRatio: 1.1
    property int timerInterval: 100       // Timer interval in milliseconds
    property int initialDelay: 300        // Initial delay before rapid increment starts
    
    signal clicked()
    
    source: buttonType === "plus" ? "qrc:/assets/icons/plus_icon.png" : "qrc:/assets/icons/minus_icon.png"
    
    Timer {
        id: continuousTimer
        interval: root.timerInterval
        repeat: true
        onTriggered: {
            root.clicked()
            root.onButtonClicked()
        }
    }
    
    // Timer for initial delay before continuous timer starts
    Timer {
        id: initialDelayTimer
        interval: root.initialDelay
        repeat: false
        onTriggered: {
            continuousTimer.start()
        }
    }
    
    MouseArea {
        id: mouseArea
        anchors.fill: parent
        enabled: root.enabled
        hoverEnabled: true
        
        onPressed: {
            // Trigger once immediately
            root.clicked()
            root.onButtonClicked()
            
            // Start the delay timer
            initialDelayTimer.start()
        }
        
        onReleased: {
            initialDelayTimer.stop()
            continuousTimer.stop()
        }
        
        onCanceled: {
            initialDelayTimer.stop()
            continuousTimer.stop()
        }
    }
    
    // Optional: Add hover effect
    states: State {
        name: "hovered"
        when: mouseArea.containsMouse
        PropertyChanges {
            target: root
            scale: scaleRatio
        }
    }
    
    transitions: Transition {
        NumberAnimation {
            properties: "scale"
            duration: 100
        }
    }
}