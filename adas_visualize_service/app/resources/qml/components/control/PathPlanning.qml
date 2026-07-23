import QtQuick 2.14

Canvas {
    id: pathPlanningCanvas
    
    property var pathPoints: []
    property real horizontalOffset: width * 0.5
    property real verticalOffset: 230
    property real maxDistance: 50
    property real meterToPixelRatio: (height * 0.7) / maxDistance
    
    onPaint: {
        var ctx = getContext("2d")
        var centerX = width - horizontalOffset
        var centerY = height - verticalOffset
        
        ctx.clearRect(0, 0, width, height)
        
        if (pathPoints.length > 0) {
            // Draw planning path
            ctx.beginPath()
            ctx.strokeStyle = "#008BFF"
            ctx.lineWidth = 1
            
            pathPoints.forEach(function(point, index) {
                var pixelX = centerX + (point.x * meterToPixelRatio)
                var pixelY = centerY - (point.y * meterToPixelRatio)
                
                if (index === 0) {
                    ctx.moveTo(pixelX, pixelY)
                } else {
                    ctx.lineTo(pixelX, pixelY)
                }
                
                // Draw point
                ctx.beginPath()
                ctx.fillStyle = "#008BFF"
                ctx.arc(pixelX, pixelY, 2, 0, 2 * Math.PI)
                ctx.fill()
            })
            
            ctx.stroke()
        }
    }
    
    onPathPointsChanged: {
        // console.log("Path points updated:", pathPoints.length)
        requestPaint()
    }
}