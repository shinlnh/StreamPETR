import QtQuick 2.14
import style 1.0

import "../../utils/VectorDrawing.js" as VectorDrawing
import "../../utils/ObjectDrawing.js" as ObjectDrawing
import "../../utils/ObjectModel.js" as ObjectModel
// import utils 1.0

Canvas {
    id: objectsCanvas
    
    property var objects: []
    property real horizontalOffset: width * 0.5
    property real verticalOffset: 230
    
    // Single maximum distance and ratio
    property real maxDistance: 50
    property real meterToPixelRatio: (height * 0.7) / maxDistance
    property real scaleVector: 2.5
    
    onPaint: {
        var ctx = getContext("2d")
        var centerX = width - horizontalOffset
        var centerY = height - verticalOffset
        
        ctx.clearRect(0, 0, width, height)
        ctx.lineWidth = 1

        var standardObjects = ObjectModel.standardizeObjects(objects);
        
        // objects.forEach(function(obj) {
        standardObjects.forEach(function(obj){
            // Convert to pixels using single ratio
            var pixelX = centerX + (obj.x * meterToPixelRatio)
            var pixelY = centerY - (obj.y * meterToPixelRatio)
            
            // // Skip objects outside the viewable area (Debug)
            // if (Math.abs(obj.x) > maxDistance || 
            //     Math.abs(obj.y) > maxDistance) {
            //     console.log("Object", obj.id, "outside viewable area:", obj.x, obj.y)
            //     return
            // }
            
            VectorDrawing.drawVelocityVector(ctx, obj, pixelX, pixelY, scaleVector);
            ObjectDrawing.drawObject(ctx, obj, pixelX, pixelY, ObjectStyles)
        })
    }

    onObjectsChanged: {
        // console.log("Objects updated:", objects.length)
        // objects.forEach(obj => 
        //     console.log("Object:", obj.id, obj.type, "position:", obj.x, obj.y, "Velocity: ", obj.vx, obj.vy)
        // )
        requestPaint()
    }
}