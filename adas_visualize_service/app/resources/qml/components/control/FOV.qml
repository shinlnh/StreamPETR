import QtQuick 2.14

Canvas {
    id: fovCanvas
    
    // FOV properties
    property int radarFOVAngle: radarViewConfig ? radarViewConfig.horizontalFov : 30        // degrees
    property int cameraFOVAngle: cameraViewConfig ? cameraViewConfig.horizontalFov : 90     // degrees
    property bool showRadar: true
    property bool showCamera: true
    
    // Coordinate properties (position and orientation)
    property var radarCoordinate: radarViewConfig ? radarViewConfig.coordinate : [2.2, 0.0, 0.5, 0.0, 0.0, 0.0]       
    property var cameraCoordinate: cameraViewConfig ? cameraViewConfig.coordinate : [2.2, 0.0, 0.5, 0.0, 0.0, 0.0]
    
    // Layout properties (sync with other components)
    property real verticalOffset: 230
    property real horizontalOffset: width * 0.5
    property real centerX: width - horizontalOffset
    property real centerY: height - verticalOffset
    
    // Distance configuration (sync with GirdRound and Objects)
    property real maxDistance: 50
    property real meterToPixelRatio: (height * 0.7) / maxDistance
    
    // Style configuration
    property color radarFOVColor: "#66B2FF"
    property color cameraFOVColor: "#FF6666"
    property real fovOpacity: 0.2
    property real fovExtendFactor: 2.0
    
    onPaint: {
        var ctx = getContext("2d")
        ctx.clearRect(0, 0, width, height)
        
        // Draw camera FOV first (lower z-index)
        if (showCamera) {
            drawFOV(ctx, cameraFOVAngle, cameraFOVColor, cameraCoordinate)
        }
        
        // Draw radar FOV above
        if (showRadar) {
            drawFOV(ctx, radarFOVAngle, radarFOVColor, radarCoordinate)
        }
    }
    
    function drawFOV(ctx, fovAngle, color, coordinate) {
        // Extract position and orientation
        var xPos = coordinate[0]
        var yPos = coordinate[1]
        var zPos = coordinate[2]
        
        // Convert roll, pitch and yaw to radians
        var roll = coordinate[3] * Math.PI / 180
        var pitch = coordinate[4] * Math.PI / 180
        var yaw = coordinate[5] * Math.PI / 180
        
        // Calculate sensor position in pixels
        var pixelX = centerX + yPos * meterToPixelRatio
        var pixelY = centerY - xPos * meterToPixelRatio
        
        // Calculate FOV points with orientation applied
        var halfAngle = (fovAngle * Math.PI / 180) / 2
        var distance = maxDistance * fovExtendFactor * meterToPixelRatio
        
        // Apply yaw rotation to the FOV (most relevant for top-down view)
        // Calculate left top point
        var leftAngle = halfAngle + yaw
        var leftX = pixelX + Math.sin(leftAngle) * distance
        var leftY = pixelY - Math.cos(leftAngle) * distance
        
        // Calculate right top point
        var rightAngle = yaw - halfAngle
        var rightX = pixelX + Math.sin(rightAngle) * distance
        var rightY = pixelY - Math.cos(rightAngle) * distance
        
        // Draw FOV as triangle
        ctx.beginPath()
        ctx.moveTo(pixelX, pixelY)
        ctx.lineTo(leftX, leftY)
        ctx.lineTo(rightX, rightY)
        ctx.closePath()
        
        // Fill with semi-transparent color
        ctx.fillStyle = Qt.rgba(
            parseInt(color.toString().substr(1, 2), 16) / 255,
            parseInt(color.toString().substr(3, 2), 16) / 255,
            parseInt(color.toString().substr(5, 2), 16) / 255,
            fovOpacity
        )
        ctx.fill()
        
        // // Only Debug: Optionally draw a small indicator for the sensor position
        // ctx.beginPath()
        // ctx.arc(pixelX, pixelY, 4, 0, 2 * Math.PI)
        // ctx.fillStyle = color
        // ctx.fill()
    }
    
    onWidthChanged: requestPaint()
    onHeightChanged: requestPaint()
    onRadarFOVAngleChanged: requestPaint()
    onCameraFOVAngleChanged: requestPaint()
    onMaxDistanceChanged: requestPaint()
    onRadarCoordinateChanged: requestPaint()
    onCameraCoordinateChanged: requestPaint()
}