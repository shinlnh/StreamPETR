import QtQuick 2.14

Canvas {
    id: girdRoundCanvas
    
    // Layout properties
    property real verticalOffset: 230
    property real horizontalOffset: width * 0.5
    property real centerX: width - horizontalOffset
    property real centerY: height - verticalOffset
    
    // Distance configuration
    property int numDivisions: 5
    property real maxDistance: 50
    property real meterToPixelRatio: (height * 0.7) / maxDistance
    
    // Optional features
    property bool show5mCircle: true
    
    // Style configurations
    readonly property color strokeColor: "#303030"
    readonly property color textColor: "#404040"
    readonly property color pointColor: "yellow"
    readonly property string fontFamily: "'Liberation Mono'"
    readonly property int fontSize: 12
    
    onPaint: {
        var ctx = getContext("2d")
        ctx.clearRect(0, 0, width, height)
        
        // Set common styles
        setCommonStyles(ctx)
        
        // // Draw 5m circle if enabled (Debug)
        // if (show5mCircle) {
        //     drawEllipticalZone(ctx, 5)
        // }
        
        // Draw ellipses
        for (var i = 1; i <= numDivisions; i++) {
            var distance = (i * maxDistance) / numDivisions
            drawEllipticalZone(ctx, distance)
        }
        
        // // Draw center guidelines (Debug)
        // drawCenterLines(ctx))
    }
    
    // Helper functions
    function getRadius(distance) {
        return distance * meterToPixelRatio
    }
    
    function setCommonStyles(ctx) {
        ctx.strokeStyle = strokeColor
        ctx.lineWidth = 1
        ctx.setLineDash([5, 5])
        ctx.fillStyle = textColor
        ctx.font = `${fontSize}px ${fontFamily}`
    }
    
    function drawEllipticalZone(ctx, distance) {
        var scaleFactor = distance / maxDistance
        var radius = getRadius(maxDistance)
        
        // Calculate scaled radiuses for ellipse
        var ratioRadius = radius * scaleFactor
        
        ctx.beginPath()
        ctx.ellipse(
            centerX - ratioRadius,
            centerY - ratioRadius,
            ratioRadius * 2,
            ratioRadius + ratioRadius
        )
        ctx.stroke()
        
        // Draw front distance marker
        // drawText(ctx, distance + "m", centerX - 15, centerY - ratioRadius - 5)
        drawTextAlongArc(ctx, distance + "m", distance, 235)
    }
    
    function drawPoint(ctx, x, y) {
        ctx.beginPath()
        ctx.arc(x, y, 3, 0, 2 * Math.PI)
        ctx.fillStyle = pointColor
        ctx.fill()
        ctx.fillStyle = textColor               // Reset fill style for text
    }
    
    function drawCenterLines(ctx) {
        var radius = getRadius(maxDistance)
        ctx.beginPath()
        ctx.moveTo(centerX, centerY - radius)
        ctx.lineTo(centerX, centerY + radius)
        ctx.moveTo(centerX - radius, centerY)
        ctx.lineTo(centerX + radius, centerY)
        ctx.stroke()
    }

    function drawText(ctx, text, x, y) {
        ctx.fillStyle = textColor
        ctx.fillText(text, x, y)
    }

    function drawTextAlongArc(ctx, text, distance, angleDegrees) {
        var radius = getRadius(distance)
        var angleRad = angleDegrees * Math.PI / 180  // Change to radians
        
        // Calculate the coordinates of the point on the arc
        var x = centerX + radius * Math.cos(angleRad)
        var y = centerY + radius * Math.sin(angleRad)
        
        ctx.save()

        // Move canvas to the point
        ctx.translate(x, y)
        // Rotate canvas
        ctx.rotate(angleRad + Math.PI/2) 

        // Draw text
        ctx.fillStyle = textColor
        ctx.fillText(text, -15, 0)
        ctx.restore()
    }
}