pragma Singleton
import QtQuick 2.14

QtObject {
    
    // Colors configurations
    readonly property var colors: ({
        ["0"]: "#FF8000",                       // White triangle
        ["1"]: "#00FF00",                       // White triangle
        ["2"]: "#D9D9D9",                       // Blue diamond
        "default": "#596EEB"
    })
    
    // Shapes configurations
    readonly property var shapes: ({
        ["0"]: {
            type: "star", // Pedestrian
            size: 12,
            xOffset: 0,
            yOffset: 0,
            points: 5
        },
        ["1"]: {
            type: "star", // Bicycle
            size: 12,
            xOffset: 0,
            yOffset: 0,
            points: 5
        },
        ["2"]: {
            type: "triangle", // Car
            size: 12,
            xOffset: 0,
            yOffset: 0
        },
        "default": {
            type: "diamond", // Obstacle
            size: 12,
            xOffset: 0,
            yOffset: 0,
            points: 5
        }
    })

    // Text configurations
    property var textStyle: ({
        font: "bold 16px 'Liberation Mono'",
        color: "#00FF00",
        xOffset: 0,
        yOffset: -2
    })

    // Sensor configurations
    readonly property var sensorXYOffset: ({
        xOffset: 4,
        yOffset: 0
    })

    readonly property var sensorsType: ({
        ["RADAR"]: {
            font: "bold 16px 'Liberation Mono'",
            type: "RADAR",
            color: "#FF000D"
        },
        ["CAMERA"]: {
            font: "bold 16px 'Liberation Mono'",
            type: "CAMERA",
            color: "#FF9933"
        },
        ["FUSION"]: {
            font: "bold 16px 'Liberation Mono'",
            type: "FUSION",
            color: "#008BFF"
        },
        "default": {
            font: "bold 16px 'Liberation Mono'",
            type: "undefined",
            color: "#FFFFFF"
        }
    })
    
    // Helper functions
    function getColor(classId) {
        return colors[classId] || colors.default
    }
    
    function getShapeConfig(classId) {
        return shapes[classId] || shapes.default
    }

    function getSensorType(type) {
        var sensors = sensorsType[type] || sensorsType.default
        var result = {}
        
        // Assign properties from sensorsType
        for (var p in sensors) {
            result[p] = sensors[p]
        }
        
        // Add common properties
        result.xOffset = sensorXYOffset.xOffset
        result.yOffset = sensorXYOffset.yOffset
        
        return result
    }
}