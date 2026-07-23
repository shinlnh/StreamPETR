import QtQuick 2.14

Canvas {
    id: lanesCanvas
    
    property var lanes: []
    property real horizontalOffset: width * 0.5
    property real verticalOffset: 230                                                       // Pixels
    property real maxDistance: 50                                                           // Maximum visible distance in meters
    property real meterToPixelRatio: (height * 0.7) / maxDistance
    property real pointRadius: 1                                                            // Size of the point dots
    property real doubleLaneOffset: 2

    onPaint: {
        var ctx = getContext("2d")
        var centerX = width - horizontalOffset
        var centerY = height - verticalOffset
        
        ctx.clearRect(0, 0, width, height)
        
        lanes.forEach(function(lane) {
            var doubleLine = 0
            var lineTypes = [0, 0]
            switch (lane.type) {  // Decide lines type, double or single
                case 0:     // Non_exist, continue to next lane
                    return
                case 1:     // Others
                    lineTypes[0] = 1
                break
                case 2:     // Broken
                break
                case 3:     // Solid
                    lineTypes[0] = 1
                break
                case 4:     // Broken - Broken
                    doubleLine = 1
                break
                case 5: {   // Broken - Solid
                    doubleLine = 1
                    lineTypes[1] = 1
                }
                break
                case 6: {   // Solid - Broken
                    doubleLine = 1
                    lineTypes[0] = 1
                }
                break
                case 7: {   // Solid - Solid
                    doubleLine = 1
                    lineTypes[0] = 1
                    lineTypes[1] = 1
                }
                break
                case 8:     // Curb
                    lineTypes[1] = 1
                break
            }

            // Draw each lane
            ctx.beginPath()
            ctx.strokeStyle = "#ffffffff"
            ctx.lineWidth = 1

            var lastPoint = {x: 0, y: 0}
            lane.points.forEach(function(point, index) {
                var pixelX = centerX + (point.x * meterToPixelRatio)
                var pixelY = centerY - (point.y * meterToPixelRatio)
                
                // // Draw point
                // ctx.save()
                // ctx.beginPath()
                // ctx.fillStyle = "#FFFFFF"  // Red color for lane points
                // ctx.arc(pixelX, pixelY, pointRadius, 0, 2 * Math.PI)
                // ctx.fill()
                // ctx.restore()
                
                if (index > 0) {
                    if (doubleLine === 1) {
                        if (lineTypes[1] === 1 || (index % 2) !== 0) {
                            ctx.moveTo(lastPoint.x - doubleLaneOffset, lastPoint.y)
                            ctx.lineTo(pixelX - doubleLaneOffset, pixelY)
                        }
                    }
                    
                    if (lineTypes[0] === 1 || (index % 2) !== 0) {
                        ctx.moveTo(lastPoint.x, lastPoint.y)
                        ctx.lineTo(pixelX, pixelY)
                    }
                }
                lastPoint.x = pixelX
                lastPoint.y = pixelY
            })

            ctx.stroke()
        })

        // Debug
        // Draw 35m rect
        // ctx.beginPath()
        // ctx.strokeStyle = "#ffffffff"
        // ctx.lineWidth = 1
        // ctx.moveTo(centerX + ( -1 * (35.0 / 2) * meterToPixelRatio), centerY - (35.0 * meterToPixelRatio))
        // ctx.lineTo(centerX + ((35.0 / 2) * meterToPixelRatio), centerY - (35.0 * meterToPixelRatio))
        // ctx.lineTo(centerX + ((35.0 / 2) * meterToPixelRatio), centerY - (0.0 * meterToPixelRatio))
        // ctx.lineTo(centerX + ( -1 * (35.0 / 2) * meterToPixelRatio), centerY - (0.0 * meterToPixelRatio))
        // ctx.lineTo(centerX + ( -1 * (35.0 / 2) * meterToPixelRatio), centerY - (35.0 * meterToPixelRatio))
        // ctx.stroke()
    }
    
    onLanesChanged: {
        // console.log("Lanes updated: ", lanes.length)
        requestPaint()
    }
}