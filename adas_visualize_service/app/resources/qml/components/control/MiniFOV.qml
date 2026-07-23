import QtQuick 2.14

Canvas {
    id: miniFovCanvas
    width: 250
    height: 250

    property int radarFOVAngle: radarViewConfig ? radarViewConfig.horizontalFov : 30
    property var radarCoordinate: radarViewConfig ? radarViewConfig.coordinate : [2.2, 0, 0, 0, 0, 0]
    
    // Display range configuration (50m-250m)
    property int minDistance: 50
    property int maxDistance: 250

    // FOV appearance properties
    property color radarFOVColor: Qt.rgba(102/255, 178/255, 255/255, 0.2)
    property real radarFOVOpacity: 0.2
    property real fovExtendFactor: 1.1  // Extend the FOV

    // Center coordinates (sensor at bottom)
    property real centerX: width * 0.5     // = 125
    property real centerY: height          // = 250

    onPaint: {
        var ctx = getContext("2d")
        ctx.clearRect(0, 0, width, height)
        
        // Draw the full FOV (0-250m)
        drawFullFOV(ctx)
        
        // Uncomment to crop the 0-50m area
        // cropZeroToFifty(ctx)
    }
    
    // Draw the complete FOV from 0 to 250m
    function drawFullFOV(ctx) {
        // Calculate yaw angle
        var yawDeg = radarCoordinate[5]
        var yaw = yawDeg * Math.PI / 180
        var halfAngle = (radarFOVAngle * Math.PI / 180) / 2
        
        // Maximum distance in pixels
        // var maxDistPx = maxDistance * fovExtendFactor

        // Extend Distance
        var extendDistance = Math.max(width, height) * 2

        // Calculate FOV triangle coordinates
        // Sensor point
        var sx = centerX
        var sy = centerY

        // Calculate left top point (at max distance)
        var leftAngle = yaw + halfAngle
        var leftTopX = centerX + Math.sin(leftAngle) * extendDistance
        var leftTopY = centerY - Math.cos(leftAngle) * extendDistance

        // Calculate right top point (at max distance)
        var rightAngle = yaw - halfAngle
        var rightTopX = centerX + Math.sin(rightAngle) * extendDistance
        var rightTopY = centerY - Math.cos(rightAngle) * extendDistance

        // Draw the FOV triangle
        ctx.save()
        
        // // Set global alpha for transparency first (before beginning the path)
        // ctx.globalAlpha = radarFOVOpacity
        
        ctx.beginPath()
        ctx.moveTo(sx, sy)           // sensor point
        ctx.lineTo(leftTopX, leftTopY)  // left top point
        ctx.lineTo(rightTopX, rightTopY)  // right top point
        ctx.closePath()

        // Fill the FOV area with transparency
        ctx.fillStyle = radarFOVColor
        ctx.fill()
        
        // Reset alpha before drawing the stroke
        ctx.globalAlpha = 1.0
        
        // // Draw FOV outline
        // ctx.strokeStyle = radarFOVColor
        // ctx.stroke()

        ctx.restore()
    }

    // Function to hide the 0-50m area if needed
    function cropZeroToFifty(ctx) {
        var hideHeight = minDistance                // = 50
        var hideY = centerY - hideHeight            // = 250-50 = 200

        ctx.save()
        ctx.fillStyle = "#232A35"
        ctx.beginPath()
        ctx.rect(0, hideY, width, height - hideY)   // Cover y=200..250
        ctx.fill()
        ctx.restore()
    }

    // Redraw on property changes
    onWidthChanged: requestPaint()
    onHeightChanged: requestPaint()
    onRadarCoordinateChanged: requestPaint()
}