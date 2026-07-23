.pragma library

function drawVelocityVector(ctx, obj, x, y, scale) {
    if (obj.vx === undefined || obj.vy === undefined) return;
    
    // Calculate vector magnitude (speed)
    const speed = Math.sqrt(obj.vx * obj.vx + obj.vy * obj.vy);
    
    // Skip if speed is 0
    if (speed === 0) return;
    
    // Calculate vector direction (angle in radians)
    const angle = Math.atan2(obj.vy, obj.vx);
    
    const length = speed * scale;
    
    // Calculate end point
    const endX = x + length * Math.cos(angle);
    const endY = y - length * Math.sin(angle);
    
    // Draw vector line
    ctx.beginPath();
    ctx.strokeStyle = "#FFFF00";
    ctx.lineWidth = 2;
    
    // Main line
    ctx.moveTo(x, y);
    ctx.lineTo(endX, endY);
    
    ctx.stroke();
}