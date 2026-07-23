.pragma library

function drawObject(ctx, obj, x, y, objectStyles) {
    ctx.beginPath()
    
    var color = objectStyles.getColor(obj.classId)
    var shapeConfig = objectStyles.getShapeConfig(obj.classId)
        
    ctx.fillStyle = color
    ctx.strokeStyle = color
    
    var angleVelocity = Math.atan2(obj.vy, obj.vx)

    // Rotate Shape (Handle orientation)
    ctx.save()
    ctx.translate(x, y)
    // ctx.rotate(angleVelocity)

    // Draw shape based on type
    var drawFunc = getShapeDrawFunction(shapeConfig.type)
    drawFunc(ctx, x + shapeConfig.xOffset, y + shapeConfig.yOffset, shapeConfig.size, shapeConfig.points)
    
    ctx.restore()

    // Fill or stroke based on shape type
    if (shapeConfig.type === "triangleEmpty") {
        ctx.stroke()
    } else {
        ctx.fill()
    }
    
    // Draw ID text (use tracking_id)
    drawObjectId(ctx, obj.tracking_id, obj.classId, x, y, obj.sensor_type, objectStyles)
}

function drawObjectId(ctx, tracking_id, classId, x, y, sensor_type, objectStyles) {
    var xOffset = 0, yOffset = 0

    var sensorConfig = objectStyles.getSensorType(sensor_type || "default");

    ctx.fillStyle = sensorConfig.color;
    ctx.font = sensorConfig.font;
    xOffset = sensorConfig.xOffset;
    yOffset = sensorConfig.yOffset;

    // Draw text
    ctx.fillText(
        tracking_id.toString(),
        x + xOffset,
        y + yOffset
    )
}

function getShapeDrawFunction(type) {
    var drawFuncs = {
        "triangle": drawTriangle,
        "triangleEmpty": drawTriangle,
        "diamond": drawDiamond,
        "star": drawStar
    }
    return drawFuncs[type] || drawTriangle
}

function drawTriangle(ctx, x, y, size) {
    var halfSize = size / 2
    ctx.beginPath()
    ctx.moveTo(0, -halfSize)
    ctx.lineTo(-halfSize, halfSize)
    ctx.lineTo(halfSize, halfSize)
    ctx.closePath()
}

function drawDiamond(ctx, x, y, size) {
    var halfSize = size / 2
    ctx.beginPath()
    ctx.moveTo(0, -halfSize)
    ctx.lineTo(halfSize, 0)
    ctx.lineTo(0, halfSize)
    ctx.lineTo(-halfSize, 0)
    ctx.closePath()
}

function drawStar(ctx, x, y, size, points) {
    // Set Default Values
    points = points || 5;
    
    var outerRadius = size / 2
    var innerRadius = outerRadius * 0.4
    var rotation = Math.PI / 2 * 3
    var step = Math.PI / points

    ctx.beginPath()
    ctx.moveTo(0, -outerRadius)

    for (var i = 0; i < points; i++) {
        var x1 = Math.cos(rotation) * outerRadius
        var y1 = Math.sin(rotation) * outerRadius
        ctx.lineTo(x1, y1)
        
        rotation += step
        x1 = Math.cos(rotation) * innerRadius
        y1 = Math.sin(rotation) * innerRadius
        ctx.lineTo(x1, y1)
        
        rotation += step
    }
    ctx.closePath()
}