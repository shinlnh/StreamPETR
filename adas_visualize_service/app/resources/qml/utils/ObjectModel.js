.pragma library

/**
 * Create a object
 * @param {Object} data - input data
 * @returns {Object} object
 */
function createObject(data) {
    // Check empty
    data = data || {};
    
    return {
        classId: data.classId !== undefined ? data.classId : -999,
        x: data.x !== undefined ? data.x : 0,
        y: data.y !== undefined ? data.y : 0,
        z: data.z !== undefined ? data.z : 0,
        
        //Tracking ID
        tracking_id: data.tracking_id !== undefined ? data.tracking_id : -999,
        
        // Velocity
        vx: data.vx !== undefined ? data.vx : 0,
        vy: data.vy !== undefined ? data.vy : 0,

        sensor_type: data.sensor_type !== undefined ? data.sensor_type : "default",
        
        // Expand
    };
}

/**
 * @param {Array} objectsArray - Input Array
 * @returns {Array} Standardized Array
 */
function standardizeObjects(objectsArray) {
    var result = [];
    
    if (!Array.isArray(objectsArray)) {
        console.warn("Invalid objects array provided to standardizeObjects");
        return result;
    }
    
    for (var i = 0; i < objectsArray.length; i++) {
        var standardObj = createObject(objectsArray[i]);
        
        // Skip invalid objects
        if (standardObj.classId != undefined || standardObj.tracking_id != undefined) {
            result.push(standardObj);
        } else {
            console.warn("Skipping invalid object at index", i);
        }
    }
    
    return result;
}