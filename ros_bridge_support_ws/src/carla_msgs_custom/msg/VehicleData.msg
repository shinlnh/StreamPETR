#
# Copyright (c) 2019 Intel Corporation.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

# VehicleData.msg

uint32 id
string type

# Relative location of every spawned vehicle
geometry_msgs/Vector3 location

# Relative velocity of every spawned vehicle
geometry_msgs/Vector3 velocity

# Absoulute velocity of every spawned vehicle
geometry_msgs/Vector3 vector_speed_kph

# Magnitude of absoulute velocity
float32 speed_kph

# Centripetal location of every spawned vehicle
float32 centripetal_location
