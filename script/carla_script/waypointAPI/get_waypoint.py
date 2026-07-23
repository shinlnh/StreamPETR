#!/usr/bin/env python
import carla
import time
import random
import threading
import numpy as np
import os
import pandas as pd

TIMEOUT = 2         # timeout to get world is 2 sec
LANE_ERROR = 1.0    # when we go to end of lane, we will try to find a new lane
VEHICLE_ID = 145
FILE_RAW_POINT = "data/raw_point.csv"
FILE_PLAN_POINT = "data/plan_point.csv"
FILE_REAL_POINT = "data/real_point.csv"

def calculate_length(point_list):
    point_arr = np.array(point_list)
    # print(f"points shape: {point_arr.shape}")
    temp = np.vstack([point_arr[0,], point_arr]) - np.vstack([point_arr, point_arr[-1, ]])
    return np.sum(np.sqrt(np.sum(temp**2, axis=1)), axis=0)


def get_waypoint_list(ref_position, carla_map, step=1.0):
    cur_waypoint = carla_map.get_waypoint(carla.Location(ref_position[0, 0], ref_position[1, 0], ref_position[2, 0]))
    return cur_waypoint.next_until_lane_end(step)


def get_my_vehicle(vehicleID, carla_world):
    # get all vehicle in the world 
    all_vehicles = carla_world.get_actors().filter('vehicle.*')
    if len(all_vehicles) == 0:
        start_time = time.time()
        while len(all_vehicles) == 0:
            all_vehicles = carla_world.get_actors().filter('vehicle.*')
            if time.time() - start_time > TIMEOUT:
                break
    if (len(all_vehicles) == 0):
        print(f"Have no vehicle in the world")
        return

    # get the vehicle have ID = 145 (default ID when spawn with scenarios)
    my_vehicle = [vehicle for vehicle in all_vehicles if vehicle.id == vehicleID]
    if (len(my_vehicle) == 0):
        print(f"Can not find my vehicle in the world")
        return
    # print(f"ID = {my_vehicle[0].id}")
    return my_vehicle[0]


def change_coordinate(my_vehicle):
    # get current direction of my vehicle and set it to the reference direction
    vehicle_transform = my_vehicle.get_transform()
    forward_vector = vehicle_transform.get_forward_vector()
    right_vector = vehicle_transform.get_right_vector()
    up_vector = vehicle_transform.get_up_vector()
    ref_direction = np.array([[right_vector.x, forward_vector.x, up_vector.x],
                              [right_vector.y, forward_vector.y, up_vector.y],
                              [right_vector.z, forward_vector.z, up_vector.z]])

    # calculate change basis matrix 
    change_basis_matrix = np.linalg.inv(ref_direction)

    # set first position of my vehical to reference point
    ref_point = np.array([[vehicle_transform.location.x],
                          [vehicle_transform.location.y],
                          [vehicle_transform.location.z]])
    return change_basis_matrix, ref_point


def write_file(points, filename=None, heading=True, truncated=False):
    if filename is None:
        print("File's name is invalid")
        return
    
    points = np.transpose(points, (1, 0))
    if truncated:
        with open(filename, "w") as file:
            pass

    with open(filename, "a") as file:
        if heading:
            file.write("x,y,z\n")
        for point in points:
            file.write(f"{point[0]},{point[1]},{point[2]}\n")


def get_waypoint_along_lane(ref_position, carla_map, step = 1.0, max_distance=2000.0):
    """
    This function will return a list of waypoint dicovered. Shape of matrix store waypoints is (3, n).
    n points, each point have 3 attribute (x, y, z)
    """
    result = []
    cur_distance = 0
    while cur_distance < max_distance:
        waypoint_list = get_waypoint_list(ref_position, carla_map, step)
        # handle exception
        if len(waypoint_list) < 2:
            print(f"lane is end at \n{ref_position}")
            break

        waypoint_list = [waypoint.transform.location for waypoint in waypoint_list]
        result += waypoint_list

        # update ref_position
        ref_position = np.array([[waypoint_list[-1].x + (random.random() * LANE_ERROR)],
                                 [waypoint_list[-1].y + (random.random() * LANE_ERROR)],
                                 [waypoint_list[-1].z + (random.random() * LANE_ERROR)]])  
        # try to get a new reference position within TIMEOUT
        if len(get_waypoint_list(ref_position, carla_map, step)) < 2:
            start_time = time.time()
            while len(get_waypoint_list(ref_position, carla_map, step)) < 2:
                ref_position = np.array([[waypoint_list[-1].x + (random.random() * LANE_ERROR)],
                                         [waypoint_list[-1].y + (random.random() * LANE_ERROR)],
                                         [waypoint_list[-1].z + (random.random() * LANE_ERROR)]])
                if time.time() - start_time > TIMEOUT:
                    print("timeout get waypoint list")
                    break
        # update length of way which is discovered
        cur_distance += calculate_length([[waypoint.x, waypoint.y, waypoint.z] for waypoint in waypoint_list])
    
    print(f"distance = {cur_distance}")
    result = np.array([[position.x, position.y, position.z] for position in result])
    return np.transpose(result, (1, 0))
    

def get_realtime_waypoint(vehicleID, carla_world, max_distance):
    cur_distance = 0
    cur_waypoint_list = []
    filename = FILE_REAL_POINT
    with open(filename, "w") as file:
        file.write(f"x,y,z\n")
    
    while cur_distance < max_distance:
        # write file after collecting 50 points
        while len(cur_waypoint_list) < 50:
            # collect waypoints
            my_vehicle = get_my_vehicle(vehicleID, carla_world)
            cur_waypoint_list += [carla_world.get_map().get_waypoint(my_vehicle.get_transform().location)]
            time.sleep(0.05)    # sampling rate: 20 Hz
        
        cur_waypoint_list = [[waypoint.transform.location.x, waypoint.transform.location.y, waypoint.transform.location.z] for waypoint in cur_waypoint_list]
        cur_distance += calculate_length(cur_waypoint_list)
        # write file
        write_file(np.transpose(np.array(cur_waypoint_list), (1, 0)), filename=filename, heading=False, truncated=False)
        cur_waypoint_list.clear()
    print(f"real distance = {cur_distance}")
    print("get waypoint completely")


def change_into_car_coordinate(ref_point, change_basis_matrix, points=None, read_file=False, filename=None):
    if not read_file:
        if points is None:
            print("Points contain 0 elements")
            return None
    else:
        if filename is None:
            print("File's name is invalid")
            return
        data = pd.read_csv(filename)
        points = np.array([data['x'], data['y'], data['z']])
    new_points = np.matmul(change_basis_matrix, points - ref_point)
    return new_points
        

def main():
    # create data folder
    os.system("mkdir -p data")

    # get expected distance from user
    expected_distance = float(input("Enter the expected distance: "))

    # default world at IP: 127.0.0.1 and port: 2000
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)    # timeout 10s

    # get world
    world = client.get_world()

    # get my vehicle
    my_vehicle = get_my_vehicle(VEHICLE_ID, world)

    # get current location and change basis matrix
    change_basis_matrix, ref_point = change_coordinate(my_vehicle)

    # start get current waypoint paralell
    thread_get_realtime_waypoint = threading.Thread(target=get_realtime_waypoint, args=(VEHICLE_ID, world, expected_distance))
    thread_get_realtime_waypoint.start()
    
    # get current map
    map = world.get_map()

    # get all waypoint along lane
    waypoint_list = get_waypoint_along_lane(ref_point, map, step=1, max_distance=expected_distance)
    raw_points = np.hstack([ref_point, waypoint_list])

    # calculate point in car coordinate system
    points = change_into_car_coordinate(ref_point, change_basis_matrix, points=raw_points, read_file=False)

    # create thread
    thread_save_plan_point = threading.Thread(target=write_file, args=(points, FILE_PLAN_POINT, True, True))
    thread_save_raw_point = threading.Thread(target=write_file, args=(raw_points, FILE_RAW_POINT, True, True))

    # start thread
    thread_save_plan_point.start()
    thread_save_raw_point.start()

    # join thread
    thread_save_plan_point.join()
    thread_save_raw_point.join()
    thread_get_realtime_waypoint.join()

    real_point = change_into_car_coordinate(ref_point, change_basis_matrix, read_file=True, filename=FILE_REAL_POINT)
    write_file(real_point, filename=FILE_REAL_POINT, heading=True, truncated=True)

if __name__ == "__main__":
    main()
    