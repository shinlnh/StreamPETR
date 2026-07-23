#ifndef CONFIG_H
#define CONFIG_H

#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <Eigen/Dense>
#include <math.h>
#include <map>
#include <memory>

#include "../../src/inc/common.h"

/**
 * @brief Represents the configuration of a sensor, including its ID, type,
 *        position, orientation, and other parameters.
 *
 * This class provides functionality to read sensor configuration from a file.
 * If the file cannot be read, the configuration will default to an empty state.
 */
class SensorConfig {
public:
    static std::map<int, std::shared_ptr<SensorConfig>> sensorConfigurations; ///< Static map for storing sensor configurations.

    std::string id; ///< Unique identifier for the sensor.
    std::string type = ""; ///< Type of the sensor.
    Eigen::Vector3f position = Eigen::Vector3f::Zero(); ///< Position of the sensor in 3D space.
    Eigen::Vector3f orientation = Eigen::Vector3f::Zero(); ///< Orientation of the sensor in 3D space.
    std::map<std::string, std::string> params; ///< Additional parameters for the sensor.

    /**
     * @brief Reads sensor configuration from the specified file.
     *
     * Parses the configuration file and sets the attributes accordingly. If the
     * file cannot be read or parsed correctly, the sensor configuration will
     * remain in its default (empty) state.
     *
     * @param filename Path to the configuration file.
     * @return True if the file was successfully read and parsed; false otherwise.
     */
    bool read_sensor_config(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            INFO("Failed to read sensor config file: %s", filename.c_str());
            return false;
        }

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) continue;

            std::istringstream iss(line);
            std::string key, value;
            std::getline(iss, key, ':');
            std::getline(iss, value);

            if (key == "type") {
                type = value;
            } else if (key == "id") {
                id = value; 
            } else if (key == "coordinate") {
                std::istringstream iss_position(value);
                std::string token;
                std::vector<double> temp;
                while (std::getline(iss_position, token, ',')) {
                    temp.push_back(std::stod(token));
                }
                if (temp.size() >= 6) {
                    position    = Eigen::Vector3f(-temp[1], temp[0], temp[2]);
                    orientation = Eigen::Vector3f( -temp[4] * M_PI / 180.0f,     // Roll
                                                    temp[3] * M_PI / 180.0f,     // Pitch
                                                    temp[5] * M_PI / 180.0f// Yaw with the 90-degree shift
                                                );
                } else {
                    file.close();
                    return false;
                }
            } else {
                params[key] = value;
            }
        }
        file.close();
        return true;
    }
};

#endif
