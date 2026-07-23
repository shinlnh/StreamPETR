#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# ==============================================================================
#  Table of Contents 
#   1. Imports
#   2. Global Variables
#   3. Global Methods
#   4. Classes
#       4.1 CarlaSyncMode
#       4.2 CarlaGame
#       4.3 Vehicle Manager
#       4.4 Lanemarkings
#   5. Main
# ==============================================================================


# ==============================================================================
# -- Imports ------------------------------------------------------------------
# ==============================================================================

import glob
import os
import sys
import math
import random

import pygame.locals

try:
    sys.path.append(glob.glob('../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

try:
    import queue
except ImportError:
    import Queue as queue

from collections import deque
from utils.buffered_saver import BufferedImageSaver
from utils.label_saver import LabelSaver
import config as cfg

# ==============================================================================
# -- Global Variables ---------------------------------------------------------
# ==============================================================================

IMAGE_WIDTH = cfg.IMAGE_WIDTH
IMAGE_HEIGHT = cfg.IMAGE_HEIGHT
FPS = cfg.FPS
FOV = cfg.FOV

isSaving = not cfg.isRender
oscillation = 0.5
pitch_rate = 1.0
yaw_rate = 8
roll_rate = 1.0
meters_per_frame = cfg.meters_per_frame
number_of_lanepoints = cfg.number_of_lanepoints
lanes = [deque(maxlen=number_of_lanepoints), 
         deque(maxlen=number_of_lanepoints), 
         deque(maxlen=number_of_lanepoints), 
         deque(maxlen=number_of_lanepoints)]


# ==============================================================================
# -- Global Methods -----------------------------------------------------------
# ==============================================================================

def reshape_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    return array


def draw_image(surface, image, blend=False):
    array = reshape_image(image)
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))


def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


# ==============================================================================
# -- CarlaSyncMode ------------------------------------------------------------
# ==============================================================================

class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context.

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', FPS)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data


# ==============================================================================
# -- Carla Game ---------------------------------------------------------------
# ==============================================================================

class CarlaGame():
    """
    Main Game Instance to execute carla simulator in pygame.
    """ 
    
    def __init__(self):
        self.actor_list = []
        pygame.init()
        
        if cfg.isRender:
            self.display = pygame.display.set_mode((IMAGE_WIDTH, IMAGE_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.font = get_font()
        self.clock = pygame.time.Clock()
        
        try:
            self.client = carla.Client('localhost', 2000)
            self.client.set_timeout(10.0)
            self.world = self.client.load_world(cfg.CARLA_TOWN)
        except RuntimeError as e:
            # print(f"Error: {e}")
            exit(1)

        self.map = self.world.get_map()
        
        # Hide all objects on the map and show street only
        # self.world.unload_map_layer(carla.MapLayer.All)
        
        self.respawn = False
        self.move_left = 0
        self.move_right = 0
        self.lanemarkings = LaneMarkings()
        self.vehiclemanager = VehicleManager()
        self.imagesaver = BufferedImageSaver(f'{cfg.output_directory}/', cfg.number_of_images, IMAGE_HEIGHT, IMAGE_WIDTH, 3, '')
        self.labelsaver = LabelSaver(cfg.label_raw)
        self.image_counter = 0

        if(cfg.isThirdPerson):
            # self.camera_transforms = [carla.Transform(carla.Location(x=-4.5, z=2.2), carla.Rotation(pitch=-14.5)),
            #                           carla.Transform(carla.Location(x=-4.0, z=2.2), carla.Rotation(pitch=-18.0))]
            self.camera_transforms = [carla.Transform(carla.Location(x=1.3, z=1.3), carla.Rotation(pitch=0.0))]
        else:
            self.camera_transforms = [carla.Transform(carla.Location(x=0.0, z=3.2), carla.Rotation(pitch=-19.5)), # camera 1
                                      carla.Transform(carla.Location(x=0.0, z=2.8), carla.Rotation(pitch=-18.5)), # camera 2
                                      carla.Transform(carla.Location(x=0.3, z=2.4), carla.Rotation(pitch=-15.0)), # camera 3
                                      carla.Transform(carla.Location(x=1.1, z=2.0), carla.Rotation(pitch=-16.5)), # camera 4
                                      carla.Transform(carla.Location(x=1.0, z=2.0), carla.Rotation(pitch=-18.5)), # camera 5
                                      carla.Transform(carla.Location(x=1.4, z=1.2), carla.Rotation(pitch=-13.5)), # camera 6
                                      carla.Transform(carla.Location(x=1.8, z=1.2), carla.Rotation(pitch=-14.5)), # camera 7
                                      carla.Transform(carla.Location(x=2.17, z=0.9), carla.Rotation(pitch=-14.5)), # camera 8
                                      carla.Transform(carla.Location(x=2.2, z=0.7), carla.Rotation(pitch=-11.5))] # camera 9   
        

    def reset_vehicle_position(self):
        """
        Resets the vehicles' position on the map. After reset the agent creates 
        a new route of (number_of_lanepoints) waypoints to follow along. 
        """
        spawn_points = self.map.get_spawn_points()
        random.seed()
        waypoint_id = random.randint(0, len(spawn_points))
        self.start_position = spawn_points[waypoint_id]
        # self.start_position = carla.Transform(carla.Location(-360, -35, 2), carla.Rotation(0, 0, 0))
        waypoint = self.map.get_waypoint(self.start_position.location)
        print(waypoint_id, waypoint)
        self.waypoint_list.append(waypoint)

        # Initialize lane deques with a fixed number of lanepoints
        for lane in lanes:
            for lanepoint in range(number_of_lanepoints):
                lane.append(None)

        for _ in range(number_of_lanepoints):
            self.lanemarkings.calculate3DLanepoints(self.client, waypoint)
            waypoint = waypoint.next(1)[0]
        
        # Randomly choose another camera setting
        camera_index = random.randint(0,len(self.camera_transforms)-1)
        
        self.camera_rgb.set_transform(self.camera_transforms[camera_index])
        self.camera_semseg.set_transform(self.camera_transforms[camera_index])
        print("Camera Index: ", camera_index)

        # Draw 3D lanemarkings in carla simulator
        if(cfg.draw3DLanes):
            for i, color in enumerate(self.lanemarkings.colormap_carla):
                for j in range(number_of_lanepoints - 1):
                    self.lanemarkings.draw_lanes(self.client, lanes[i][j], lanes[i][j + 1], self.lanemarkings.colormap_carla[color])


    def change_waypoint(self):
        """ Move the last waypoint in the list to reflect user input """
        if self.move_left or self.move_right:
            # Calculate the perpendicular vector 
            last_waypoint = self.waypoint_list[-1]
            orientationVec = last_waypoint.transform.get_forward_vector()
            length = math.sqrt(math.pow(orientationVec.y, 2) + math.pow(orientationVec.x, 2))
            perVec = carla.Location(orientationVec.y,-orientationVec.x, 0) / length * last_waypoint.lane_width

            # Move 
            new_waypoint = last_waypoint.transform.location + perVec * (self.move_left - self.move_right)
            new_waypoint = self.map.get_waypoint(new_waypoint)

            # Make sure new waypoint is nearby
            distance = last_waypoint.transform.location.distance(new_waypoint.transform.location)
            if 0.5 <= (distance / last_waypoint.lane_width) <= 3.0:
                self.waypoint_list[-1] = new_waypoint
        self.move_left = self.move_right = 0


    def detect_lanemarkings(self, new_waypoint, image_semseg):
        """
        Calculate and show all lanes on the road.

        Args:
            new_waypoint: carla.Waypoint. Calculate all the lanemarkings based on the new_waypoint, which is the last element from the waypoint_list.
            image_semseg: numpy array. Filter lanepoints with semantic segmentation camera. 
        """
        lanes_list = []      # filtered 2D-Points
        x_lanes_list = []    # only x values of lanes
        waypoint = self.waypoint_list[0]
        for _ in range(number_of_lanepoints):
            lanes_3Dcoords, lanes_type = self.lanemarkings.calculate3DLanepoints(self.client, waypoint)
            waypoint = waypoint.next(1)[0]
        
        for i, lane_3Dcoords in enumerate(lanes_3Dcoords):
            lane = self.lanemarkings.calculate2DLanepoints(self.camera_rgb, lane_3Dcoords)
            lane = self.lanemarkings.calculateYintersections(lane)
            lane = self.lanemarkings.filter2DLanepoints(lane, image_semseg)
            x_lane = [point[0] for point in lane]

            lanes_list.append(lane)
            x_lanes_list.append(x_lane)
            lanes_type[i] = lanes_type[i] if any([x != -2 for x in x_lane]) else 0
        
        return lanes_list, x_lanes_list, lanes_type

    
    def render_display(self, image, image_semseg, lanes_list, render_lanes=True):
        """
        Renders the images captured from both cameras and shows it on the
        pygame display

        Args:
            image: numpy array. Shows the 3-channel numpy imagearray on the pygame display.
            image_semseg: numpy array. Shows the semantic segmentation image on the pygame display.
        """
        def line_(y):
            y[0] += 20
            return y[0] - 20
        
        # Draw the display.
        draw_image(self.display, image)
        #draw_image(self.display, image_semseg, blend=True)
        
        # Draw lanepoints of every lane on pygame window
        if(render_lanes):
            for i, color in enumerate(self.lanemarkings.colormap):
                if(lanes_list[i]):
                    for j in range(len(lanes_list[i])):
                        if 0 <= lanes_list[i][j][0] <= IMAGE_WIDTH:
                            pygame.draw.circle(self.display, self.lanemarkings.colormap[color], lanes_list[i][j], 3, 2)
        
        y = [10]
        self.display.blit(self.font.render('% 5d FPS ' % self.clock.get_fps(), True, (255, 255, 255)), (8, line_(y)))
        self.display.blit(self.font.render('Dataset % 2d ' % self.imagesaver.reset_count, True, (255, 255, 255)), (20, line_(y)))
        self.display.blit(self.font.render('Map: ' + cfg.CARLA_TOWN, True, (255, 255, 255)), (20, line_(y)))
        self.display.blit(self.font.render('Speed: % 2.1f' % meters_per_frame, True, (255, 255, 255)), (20, line_(y)))
        self.display.blit(self.font.render('Oscillation: % 1.2f' % oscillation, True, (255, 255, 255)), (20, line_(y)))
        self.display.blit(self.font.render('Pitch rate: % 2.1f' % pitch_rate, True, (255, 255, 255)), (20, line_(y)))
        self.display.blit(self.font.render('Yaw rate: % 2d' % yaw_rate, True, (255, 255, 255)), (20, line_(y)))
        self.display.blit(self.font.render('Roll rate: % 2.1f' % roll_rate, True, (255, 255, 255)), (20, line_(y)))
        y = [190]
        self.display.blit(self.font.render('-------- CONTROL --------', True, (200, 200, 255)), (30, line_(y)))
        self.display.blit(self.font.render('Speed:            Up/Down Arrow', True, (255, 255, 255)), (20, line_(y)))
        self.display.blit(self.font.render('Oscillation:      Left/Right Arrow', True, (255, 255, 255)), (20, line_(y)))
        self.display.blit(self.font.render('Pitch rate:       Ctrl + Up/Down Arrow', True, (255, 255, 255)), (20, line_(y)))
        self.display.blit(self.font.render('Yaw rate:         Ctrl + Left/Right Arrow', True, (255, 255, 255)), (20, line_(y)))
        self.display.blit(self.font.render('Roll rate:        Ctrl + Shift + Left/Right Arrow', True, (255, 255, 255)), (20, line_(y)))
        self.display.blit(self.font.render('Respawn ego:      R', True, (255, 255, 255)), (20, line_(y)))
        self.display.blit(self.font.render('Change lane:      A/D', True, (255, 255, 255)), (20, line_(y)))
        self.display.blit(self.font.render('Toogle Record:    P', True, (255, 255, 255)), (20, line_(y)))
        self.display.blit(self.font.render('Respawn surround: P', True, (255, 255, 255)), (20, line_(y)))
        self.display.blit(self.font.render('Toggle surround:  Numpad 1->9 (Individual)', True, (255, 255, 255)), (20, line_(y)))
        self.display.blit(self.font.render('                  Numpad 0    (All) ', True, (255, 255, 255)), (20, line_(y)))
        pygame.draw.circle(self.display, (255, 30, 30) if isSaving else (60, 60, 60), (IMAGE_WIDTH - 50, 50), 30, 10)
        
        pygame.display.flip()
        

    def execute(self):
        try:
            self.initialize()

            with CarlaSyncMode(self.world, self.camera_rgb, self.camera_semseg, fps=FPS) as self.sync_mode:
                while True:
                    if self.stop_saving():
                        return
                    if self.parse_input() == "quit":
                        return
                    self.on_gameloop()

        finally:
            print('Saving files...')
            self.imagesaver.save()
            self.labelsaver.close_file()
            
            print('Destroying actors and cleaning up.')
            for actor in self.actor_list:
                actor.destroy()

            self.vehiclemanager.destroy()
            
            pygame.quit()
            print('Done.')
            
            
    def initialize(self):
        """
        Initialize the world and spawn vehicles, cameras and saver.
        """
        self.blueprint_library = self.world.get_blueprint_library()

        self.start_position = random.choice(self.map.get_spawn_points())
        self.vehicle = self.world.spawn_actor(random.choice(self.blueprint_library.filter('vehicle.tesla.model3')), self.start_position)
        self.actor_list.append(self.vehicle)
        self.vehicle.set_simulate_physics(False)
        
        self.camera_transform = self.camera_transforms[random.randint(0,len(self.camera_transforms)-1)]
        
        # Spawn rgb-cam and attach to vehicle
        self.bp_camera_rgb = self.blueprint_library.find('sensor.camera.rgb')
        self.bp_camera_rgb.set_attribute('image_size_x', f'{IMAGE_WIDTH}')
        self.bp_camera_rgb.set_attribute('image_size_y', f'{IMAGE_HEIGHT}')
        self.bp_camera_rgb.set_attribute('fov', f'{FOV}')
        self.camera_rgb_spawnpoint = self.camera_transform
        self.camera_rgb = self.world.spawn_actor(self.bp_camera_rgb, self.camera_rgb_spawnpoint, attach_to=self.vehicle)
        self.actor_list.append(self.camera_rgb)
        
        # Spawn semseg-cam and attach to vehicle
        self.bp_camera_semseg = self.blueprint_library.find('sensor.camera.semantic_segmentation')
        self.bp_camera_semseg.set_attribute('image_size_x', f'{IMAGE_WIDTH}')
        self.bp_camera_semseg.set_attribute('image_size_y', f'{IMAGE_HEIGHT}')
        self.bp_camera_semseg.set_attribute('fov', f'{FOV}')
        self.camera_semseg_spawnpoint = self.camera_transform
        self.camera_semseg = self.world.spawn_actor(self.bp_camera_semseg, self.camera_semseg_spawnpoint, attach_to=self.vehicle)
        self.actor_list.append(self.camera_semseg)
        

        # Create n waypoints to have an initial route for the vehicle
        self.waypoint_list = deque(maxlen=2)
        self.reset_vehicle_position()
    
    
    def on_gameloop(self):
        """
        Determines the logic of movement and what should happen every frame. 
        Also adds an image to the image saver and the corresponding label to the label saver, if the frame is meant to be saved. 
        In the actual implementation the points, which will be saved, are drawn to the screen on runtime (not on the saved images).
        """
        self.clock.tick()
        
        # Advance the simulation and wait for the data.
        snapshot, image_rgb, image_semseg = self.sync_mode.tick(timeout=1.0)
        
        # Change lane if keyboard is pressed.
        self.change_waypoint()

        # Move own vehicle to the next waypoint
        new_waypoint = self.vehiclemanager.move_agent(self.vehicle, self.waypoint_list)
        
        # Move neighbor vehicles with the same speed as the own vehicle
        self.vehiclemanager.move_vehicles(self.waypoint_list)
        
        # Detect if junction is ahead
        self.vehiclemanager.detect_junction(self.waypoint_list)
        
        # Convert and reshape image from Nx1 to shape(720, 1280, 3)
        image_semseg.convert(carla.ColorConverter.CityScapesPalette)
        image_semseg = reshape_image(image_semseg)
        
        # Calculate all the lanes with the helper class 'LaneMarkings'
        lanes_list, x_lanes_list, lanes_type = self.detect_lanemarkings(new_waypoint, image_semseg)
        
        # Draw all 3D lanes in carla simulator
        if cfg.draw3DLanes:
            for i, color in enumerate(self.lanemarkings.colormap_carla):
                    self.lanemarkings.draw_lanes(self.client, lanes[i][-1], lanes[i][-2], self.lanemarkings.colormap_carla[color])
        
        # Render the pygame display and show the lanes accordingly
        if cfg.isRender:
            self.render_display(image_rgb, image_semseg, lanes_list)

        # Save images using buffered imagesaver
        if isSaving:
            if((not cfg.junctionMode and self.vehiclemanager.junctionInSightCounter <= 0) or cfg.junctionMode):
                self.imagesaver.add_image(image_rgb.raw_data, 'CameraRGB')
                self.labelsaver.add_label(x_lanes_list, lanes_type)
        self.image_counter += 1
        if (cfg.images_until_respawn > 0) and (self.image_counter % cfg.images_until_respawn == 0) or self.respawn:
            self.respawn = False
            self.reset_vehicle_position()


    def stop_saving(self):
        """
        After collecting more than n .npy files, stop saving and close the game window

        Returns True if we collected more than 100 .npy files
        """
        if(self.imagesaver.reset_count > cfg.total_number_of_imagesets - 1):
            print("Data collected...")
            return True
        
        return False


    def parse_input(self):
        global isSaving
        global meters_per_frame
        global oscillation
        global pitch_rate
        global yaw_rate
        global roll_rate

        if not cfg.isRender:  # return if render windows does not exist
            return None
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return "quit"
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_q and pygame.key.get_mods() & pygame.KMOD_CTRL:
                    return "quit"
                elif event.key == pygame.K_p:
                    isSaving = not isSaving

                # Ego car control
                elif event.key == pygame.K_r:
                    self.respawn = True
                elif event.key == pygame.K_a:
                    self.move_left = 1
                elif event.key == pygame.K_d:
                    self.move_right = 1

                # Surround vehicles control
                elif event.key == pygame.K_v:
                    self.vehiclemanager.spawn_vehicles(self.world)
                elif event.key == pygame.K_KP0:
                    self.vehiclemanager.enable_vehicles ^= 1
                elif pygame.K_KP1 <= event.key <= pygame.K_KP9:
                    vehicle_id = event.key - pygame.K_KP1
                    self.vehiclemanager.toggle_vehicle(vehicle_id)

        
        keys = pygame.key.get_pressed()
        mods = pygame.key.get_mods()
        # Movement oscillation control
        if keys[pygame.K_UP] and mods & pygame.KMOD_CTRL:
            pitch_rate = min(pitch_rate + 0.2, 10)
        elif keys[pygame.K_DOWN] and mods & pygame.KMOD_CTRL:
            pitch_rate = max(pitch_rate - 0.2, 0)
        elif keys[pygame.K_UP]:
            meters_per_frame = min(meters_per_frame + 0.5, 10.0)
        elif keys[pygame.K_DOWN]:
            meters_per_frame = max(meters_per_frame - 0.5, 0.0)
        elif keys[pygame.K_RIGHT] and mods & pygame.KMOD_CTRL and mods & pygame.KMOD_SHIFT:
            roll_rate = min(roll_rate + 0.2, 10)
        elif keys[pygame.K_LEFT] and mods & pygame.KMOD_CTRL and mods & pygame.KMOD_SHIFT:
            roll_rate = max(roll_rate - 0.2, 0)
        elif keys[pygame.K_RIGHT] and mods & pygame.KMOD_CTRL:
            yaw_rate = min(yaw_rate + 1, 30)
        elif keys[pygame.K_LEFT] and mods & pygame.KMOD_CTRL:
            yaw_rate = max(yaw_rate - 1, 0)
        elif keys[pygame.K_RIGHT]:
            oscillation = min(oscillation + 0.05, 2.0)
        elif keys[pygame.K_LEFT]:
            oscillation = max(oscillation - 0.05, 0.0)
                

        return None


# ==============================================================================
# -- Vehicle Manager ----------------------------------------------------------
# ==============================================================================

class VehicleManager():
    """
    Helper class to spawn and manage neighbor vehicles
    """
    def __init__(self):
        self.osc_counter = 0.0
        self.pitch_counter = 0.0
        self.yaw_counter = 0.0
        self.enable_vehicles = True
        self.landfill = carla.Transform(carla.Location(-1000,-1000, 0))
        self.vehicles_list = []
        
        if not cfg.junctionMode:
            self.junctionInSightCounter = number_of_lanepoints
    
    
    # NOTE: This function require heavy modification (or removed entirely) since its mechanism is clunky and ineffective.
    def detect_junction(self, waypoint_list):
        """
        Detects, whether the actual image contains a junction. 
        Because of a long multi-lane highway in Town04, the argument here is different to other towns. Carla doesn't distinguish junctions, 
        so a highway access is a junction as well. To avoid these normal pictures from being filtered out, we just detect multiple choices of drivable directions as a junction.
        While this is possible in Town04, other towns have to many special cases, where errors would be included in the dataset, so we have to check junctions the normal way.
        The junctionInSightCounter describes how many images from the actual one shouldn't be saved. 

        Args:
            waypoint_list: list. List of waypoints where the vehicle will be next.
        """
        if not cfg.junctionMode:
            if cfg.CARLA_TOWN == ('Town04' or 'Town06'):
                res =[i for i,v in enumerate(waypoint_list) if len(v.next(meters_per_frame + 0.1))>1] # add 0.1 to prevent fault when speed is 0
                if res:
                    junction_argument = res[0] < number_of_lanepoints - 15
                else:
                    junction_argument = False
                #junction_argument = len(self.potential_new_waypoints) > 1
                offset = 20
            else:
                res =[i for i,v in enumerate(waypoint_list) if v.is_junction]
                if res:
                    junction_argument = res[0] < number_of_lanepoints - 15
                else:
                    junction_argument = False

                offset = 0
            # Junctions are not included in the set of trainingsdata, so we jump over every picture where a junction is in sight 
            if junction_argument:
                self.junctionInSightCounter = number_of_lanepoints - 20 + offset
            else:
                self.junctionInSightCounter -= 1 
            
    
    def move_agent(self, vehicle, waypoint_list):
        """
        Move the own agent along the given waypoints in waypoint_list.
        After we moved to the first position of the list, we need to look 
        out for a new future waypoint to append to the list, which is chosen 
        randomly. The waypoint_list always contains a fixed number of waypoints.

        Args:
            vehicle: carla.Actor. Vehicle to move along the given waypoints.
            waypoint_list: list. List of waypoints to move the vehicle along.

        Returns:
            new_waypoint: carla.Waypoint. Append the new_waypoint to the waypoint_list every tick.
        """
        global oscillation
        global pitch_rate, yaw_rate, roll_rate

        # Calculate next location and angle, then move the car
        self.osc_counter = (self.osc_counter + 0.08) % (2*math.pi)
        self.pitch_counter = (self.pitch_counter + 0.2) % (2*math.pi)
        self.yaw_counter = (self.yaw_counter + 0.08) % (2*math.pi)

        location = carla.Location(waypoint_list[-1].transform.location)
        location += waypoint_list[-1].transform.get_right_vector() * oscillation * (2/math.pi * math.asin(math.sin(self.osc_counter)))
        rotation = carla.Rotation(pitch = waypoint_list[-1].transform.rotation.pitch + pitch_rate * math.sin(self.pitch_counter), 
                                  yaw = waypoint_list[-1].transform.rotation.yaw + yaw_rate * math.sin(self.yaw_counter), 
                                  roll = waypoint_list[-1].transform.rotation.roll + roll_rate * random.random())

        vehicle.set_transform(carla.Transform(location, rotation))
        
        # Finally look for a new future waypoint to append to the list and show the lanepoints accordingly
        if meters_per_frame > 0.0 :
            self.potential_new_waypoints = waypoint_list[-1].next(meters_per_frame)
            new_waypoint = random.choice(self.potential_new_waypoints)
        else:
            new_waypoint = waypoint_list[-1]
        waypoint_list.append(new_waypoint)
        
        return new_waypoint
        
        
    def spawn_vehicles(self, world):
        """
        Spawns random vehicles on the map.
        Each vehicle is assiged a position relatively to the ego car.

        Args:
            world: carla.World. Get the spawnpoints from the world object.
        """
        # Create vehicle info template
        vehicles_template = [{"vehicle": None, "side": "l", "dist": random.randint(1,7), "enabled": False, "counter": 0.0},
                             {"vehicle": None, "side": "m", "dist": random.randint(5,7), "enabled": False, "counter": 0.0},
                             {"vehicle": None, "side": "r", "dist": random.randint(1,7), "enabled": False, "counter": 0.0},
                             {"vehicle": None, "side": "l", "dist": random.randint(10,15), "enabled": False, "counter": 0.0},
                             {"vehicle": None, "side": "m", "dist": random.randint(10,15), "enabled": False, "counter": 0.0},
                             {"vehicle": None, "side": "r", "dist": random.randint(10,15), "enabled": False, "counter": 0.0},
                             {"vehicle": None, "side": "l", "dist": random.randint(20,30), "enabled": False, "counter": 0.0},
                             {"vehicle": None, "side": "m", "dist": random.randint(20,30), "enabled": False, "counter": 0.0},
                             {"vehicle": None, "side": "r", "dist": random.randint(20,30), "enabled": False, "counter": 0.0}]
        
        # Destroy existing vehicle before respawn and copy current status
        if len(self.vehicles_list) > 0:
            self.destroy()
            for i in range(len(vehicles_template)):
                vehicles_template[i]["enabled"] = self.vehicles_list[i]["enabled"]
                vehicles_template[i]["counter"] = self.vehicles_list[i]["counter"]

        # Get car blueprints
        spawn_points = world.get_map().get_spawn_points()
        vehicles = world.get_blueprint_library().filter('vehicle.*')
        cars = [vehicle for vehicle in vehicles if int(vehicle.get_attribute('number_of_wheels')) == 4]
        
        # Spawn cars and assign them to different positions
        for i in range(len(vehicles_template)):
            car = random.choice(cars)
            neighbor_vehicle = world.spawn_actor(car, spawn_points[i])
            neighbor_vehicle.set_simulate_physics(False)
            neighbor_vehicle.set_transform(self.landfill)
            vehicles_template[i]["vehicle"] = neighbor_vehicle

        self.vehicles_list = vehicles_template
            

    def move_vehicles(self, waypoint_list):
        """
        Move the neighbor vehicles with the same speed as the own vehicle. 
        Methods are encapsulated in nested function to prevent calling
        them from outside, since they only should be called all together 
        and not individually.

        Args:
            waypoint_list: list. Access the waypoints from the waypoint_list to check lane information.
            frame_counter: int. Use the frame_counter to determine, when to randomly swap the neighbor vehicles. 
        """
        def move_(vehicle):
            next_transform = self.landfill
            if not vehicle["enabled"] or not self.enable_vehicles:  # hide if not enabled
                vehicle["vehicle"].set_transform(next_transform)
                return

            vehicle["counter"] += (random.random() / 5)
            dist = vehicle["dist"] + math.sin(vehicle["counter"]) * (vehicle["dist"] / 8)
            next_waypoint = waypoint_list[0].next(dist)[0]
            if(vehicle["side"] == "m"):
                if(next_waypoint):
                    next_transform = carla.Transform(next_waypoint.transform.location, 
                                                    next_waypoint.transform.rotation)

            elif(vehicle["side"] == "l"):
                if(next_waypoint.get_left_lane() and 
                   next_waypoint.get_left_lane().lane_type == carla.LaneType.Driving and 
                   next_waypoint.get_left_lane().transform.rotation == next_waypoint.transform.rotation):
                    next_transform = carla.Transform(next_waypoint.get_left_lane().transform.location, 
                                                    next_waypoint.get_left_lane().transform.rotation)

            elif(vehicle["side"] == "r"):
                if(next_waypoint.get_right_lane() and 
                   next_waypoint.get_right_lane().lane_type == carla.LaneType.Driving and 
                   next_waypoint.get_right_lane().transform.rotation == next_waypoint.transform.rotation):
                    next_transform = carla.Transform(next_waypoint.get_right_lane().transform.location, 
                                                    next_waypoint.get_right_lane().transform.rotation)
            
            vehicle["vehicle"].set_transform(next_transform)
        
        # Function start here
        for vehicle in self.vehicles_list:
            move_(vehicle)


    def toggle_vehicle(self, id):
        if 0 <= id < len(self.vehicles_list):
            self.vehicles_list[id]["enabled"] ^= 1


    def destroy(self):
        for vehicle in self.vehicles_list:
            if vehicle["vehicle"] is not None:
                vehicle["vehicle"].destroy()


# ==============================================================================
# -- Lane Markings ------------------------------------------------------------
# ==============================================================================

class LaneMarkings():
    """
    Helper class to detect and draw lanemarkings in carla.
    """
    def __init__(self):
        self.colormap = {'green': (0, 255, 0),
                         'red': (255, 0, 0),
                         'yellow': (255, 255, 0),
                         'blue': (0, 0, 255)}
        
        self.colormap_carla = {'green': carla.Color(0, 255, 0),
                               'red': carla.Color(255, 0, 0),
                               'yellow': carla.Color(255, 255, 0),
                               'blue': carla.Color(0, 0, 255)}

        # Intrinsic camera matrix needed to convert 3D-world coordinates to 2D-imagepoints
        f = IMAGE_WIDTH / (2 * math.tan(FOV * math.pi / 360))
        c_x = IMAGE_WIDTH/2
        c_y = IMAGE_HEIGHT/2
        
        self.cameraMatrix  = np.float32([[f, 0, c_x],
                                         [0, f, c_y],
                                         [0, 0, 1]])
    
    
    def draw_points(self, client, point):
        client.get_world().debug.draw_point(point + carla.Location(z=0.05), size=0.05, life_time=number_of_lanepoints/FPS, persistent_lines=False)    
    
    
    def draw_lanes(self, client, point0, point1, color):
        if(point0 and point1):
            client.get_world().debug.draw_line(point0 + carla.Location(z=0.05), point1 + carla.Location(z=0.05), thickness=0.05, 
                color=color, life_time=number_of_lanepoints/FPS, persistent_lines=False)


    def convertLaneType(self, carlaType):
        if carlaType is carla.LaneMarkingType.NONE:
            return 0
        elif carlaType is carla.LaneMarkingType.Broken:
            return 2
        elif carlaType is carla.LaneMarkingType.Solid:
            return 3
        elif carlaType is carla.LaneMarkingType.BrokenBroken:
            return 4
        elif carlaType is carla.LaneMarkingType.BrokenSolid:
            return 5
        elif carlaType is carla.LaneMarkingType.SolidBroken:
            return 6
        elif carlaType is carla.LaneMarkingType.SolidSolid:
            return 7
        elif carlaType is carla.LaneMarkingType.Curb:
            return 8
        else: # Others lane type
            return 1


    def calculate3DLanepoints(self, client, middle_lane):
        """
        Calculates the 3-dimensional position of the lane from the middle_lane from the informations given. 
        The information analyzed is the lanemarking type, the lane type and for the calculation the 
        middle_lane (the middle of the actual lane), the lane width and the driving direction of the middle_lane. 
        In addition to the Lanemarking position, the function does also calculate the position of respectively 
        adjacent lanes. The actual implementation includes contra flow lanes.
        
        Args:
            client: carla.Client. The client is used to draw 3-dimensional middle_lanes.
            middle_lane: carla.Waypoint. The middle_lane, of wich the lanemarkings should be calculated.

        Returns:
            lanes: list of 4 lanelists. 3D-positions of every lanemarking of the given middle_lanes actual lane, and corresponding neighbour lanes if they exist.
        """
        lanes_type = [None, None, None, None]
        left_lane = middle_lane.get_left_lane()
        right_lane = middle_lane.get_right_lane()

        orientationVec = middle_lane.transform.get_forward_vector()
        length = math.sqrt(orientationVec.y*orientationVec.y+orientationVec.x*orientationVec.x)
        abVec = carla.Location(orientationVec.y, -orientationVec.x, 0) / length * 0.5*middle_lane.lane_width
        middle_left_marking = middle_lane.transform.location + abVec
        middle_right_marking = middle_lane.transform.location - abVec
        
        if(cfg.junctionMode):
            # Lanemarkings of current lane (middle lane)
            if(middle_lane.left_lane_marking.type != carla.LaneMarkingType.NONE):
                lanes[0].append(middle_left_marking)
                lanes_type[0] = self.convertLaneType(middle_lane.left_lane_marking.type)
            else:
                lanes[0].append(None)
                lanes_type[0] = 0

            if(middle_lane.right_lane_marking.type != carla.LaneMarkingType.NONE):
                lanes[1].append(middle_right_marking)
                lanes_type[1] = self.convertLaneType(middle_lane.right_lane_marking.type)
            else:
                lanes[1].append(None)  
                lanes_type[1] = 0
        
            # Remaining outer lane markings (left and right).
            if(left_lane and left_lane.left_lane_marking.type != carla.LaneMarkingType.NONE):
                outer_left_marking  = middle_lane.transform.location + 3 * abVec
                lanes[2].append(outer_left_marking)
                lanes_type[2] = self.convertLaneType(left_lane.left_lane_marking.type)
                #draw_points(client, outer_middle_left_marking)
            else:
                lanes[2].append(None)
                lanes_type[2] = 0
    
            if(right_lane and right_lane.right_lane_marking.type != carla.LaneMarkingType.NONE):
                outer_right_marking = middle_lane.transform.location - 3 * abVec
                lanes[3].append(outer_right_marking)
                lanes_type[3] = self.convertLaneType(right_lane.right_lane_marking.type)
                #draw_points(client, outer_middle_right_marking)
            else:
                lanes[3].append(None)
                lanes_type[3] = 0
            
            #draw_points(client, middle_left_marking)
            #draw_points(client, middle_right_marking)

        else:
            if middle_left_marking:
                lanes[0].append(middle_left_marking)
                lanes_type[0] = self.convertLaneType(middle_lane.left_lane_marking.type)
            else:
                lanes[0].append(None)
                lanes_type[0] = 0

            if middle_right_marking:
                lanes[1].append(middle_right_marking)
                lanes_type[1] = self.convertLaneType(middle_lane.right_lane_marking.type)
            else:
                lanes[1].append(None) 
                lanes_type[1] = 0
        
            # Calculate remaining outer lanes (left and right).
            if(left_lane and left_lane.lane_type == carla.LaneType.Driving):
                outer_left_marking  = middle_lane.transform.location + 3 * abVec
                lanes[2].append(outer_left_marking)
                lanes_type[2] = self.convertLaneType(left_lane.left_lane_marking.type)
                #draw_points(client, outer_middle_left_marking)
            else:
                lanes[2].append(None)
                lanes_type[2] = 0
                
            if(right_lane and right_lane.lane_type == carla.LaneType.Driving):
                outer_right_marking = middle_lane.transform.location - 3 * abVec
                lanes[3].append(outer_right_marking)
                lanes_type[3] = self.convertLaneType(right_lane.right_lane_marking.type)
                #draw_points(client, outer_middle_right_marking)
            else:
                lanes[3].append(None)
                lanes_type[3] = 0
            
            #draw_points(client, middle_left_marking)
            #draw_points(client, middle_right_marking)
        
        return lanes, lanes_type


    def calculate2DLanepoints(self, camera_rgb, lane_list):
        """
        Transforms the 3D-lanepoint coordinates to 2D-imagepoint coordinates. If there's a huge hole in the list (None values),
        we need to split the list into two flat_lane_lists to make sure, the lanepoints are calculated and shown properly.
        
        Args:
            camera_rgb: carla.Actor. Get the camera_rgb actor to calculate the extrinsic matrix with the help of inverse matrix.
            lane_list: list. List of a lane, which contains its 3D-lanepoint coordinates x, y and z.

        Returns:
            List of 2D-points, where the elements of the list are tuples. Each tuple contains an x and y value.
        """
        flat_lane_list_a = []
        flat_lane_list_b = []
        lane_list = list(filter(lambda x: x!= None, lane_list))
        
        if lane_list:    
            last_lanepoint = lane_list[0]
            
        for lanepoint in lane_list:
            if(lanepoint and last_lanepoint):
                # Draw outer lanes not on junction
                distance = math.sqrt(math.pow(lanepoint.x-last_lanepoint.x ,2)
                                    +math.pow(lanepoint.y-last_lanepoint.y ,2)
                                    +math.pow(lanepoint.z-last_lanepoint.z ,2))
            
                # Check of there's a hole in the list
                if distance > 3:
                    flat_lane_list_b = flat_lane_list_a
                    flat_lane_list_a = []
                    last_lanepoint = lanepoint
                    continue
                
                last_lanepoint = lanepoint
                flat_lane_list_a.append([lanepoint.x, lanepoint.y, lanepoint.z, 1.0])

            else:
                # Just append a "Null" value
                flat_lane_list_a.append([None, None, None, None])
        
        if flat_lane_list_a:
            world_points = np.float32(flat_lane_list_a).T
            
            # This (4, 4) matrix transforms the points from world to sensor coordinates.
            world_2_camera = np.array(camera_rgb.get_transform().get_inverse_matrix())
            
            # Transform the points from world space to camera space.
            sensor_points = np.dot(world_2_camera, world_points)
            
            # Now we must change from UE4's coordinate system to a "standard" one
            # (x, y ,z) -> (y, -z, x)
            point_in_camera_coords = np.array([sensor_points[1],
                                               sensor_points[2] * -1,
                                               sensor_points[0]])
            
            # Finally we can use our intrinsic matrix to do the actual 3D -> 2D.
            points_2d = np.dot(self.cameraMatrix, point_in_camera_coords)
            
            # Remember to normalize the x, y values by the 3rd value.
            points_2d = np.array([points_2d[0, :] / points_2d[2, :],
                                  points_2d[1, :] / points_2d[2, :],
                                  points_2d[2, :]])
            
            # visualize everything on a screen, the points that are out of the screen
            # must be discarted, the same with points behind the camera projection plane.
            points_2d = points_2d.T
            points_in_canvas_mask = (points_2d[:, 0] > 0.0) & (points_2d[:, 0] < IMAGE_WIDTH) & \
                                    (points_2d[:, 1] > 0.0) & (points_2d[:, 1] < IMAGE_HEIGHT) & \
                                    (points_2d[:, 2] > 0.0)
            
            points_2d = points_2d[points_in_canvas_mask]
            
            # Extract the screen coords (xy) as integers.
            x_coord = points_2d[:, 0].astype(np.int)
            y_coord = points_2d[:, 1].astype(np.int)
        else:
            x_coord = []
            y_coord = []

        if flat_lane_list_b:
            world_points = np.float32(flat_lane_list_b).T
            
            # This (4, 4) matrix transforms the points from world to sensor coordinates.
            world_2_camera = np.array(camera_rgb.get_transform().get_inverse_matrix())
            
            # Transform the points from world space to camera space.
            sensor_points = np.dot(world_2_camera, world_points)
            
            # Now we must change from UE4's coordinate system to a "standard" one
            # (x, y ,z) -> (y, -z, x)
            point_in_camera_coords = np.array([sensor_points[1],
                                               sensor_points[2] * -1,
                                               sensor_points[0]])
            
            # Finally we can use our intrinsic matrix to do the actual 3D -> 2D.
            points_2d = np.dot(self.cameraMatrix, point_in_camera_coords)
            
            # Remember to normalize the x, y values by the 3rd value.
            points_2d = np.array([points_2d[0, :] / points_2d[2, :],
                                  points_2d[1, :] / points_2d[2, :],
                                  points_2d[2, :]])
            
            # visualize everything on a screen, the points that are out of the screen
            # must be discarted, the same with points behind the camera projection plane.
            points_2d = points_2d.T
            points_in_canvas_mask = (points_2d[:, 0] > 0.0) & (points_2d[:, 0] < IMAGE_WIDTH) & \
                                    (points_2d[:, 1] > 0.0) & (points_2d[:, 1] < IMAGE_HEIGHT) & \
                                    (points_2d[:, 2] > 0.0)
            
            points_2d = points_2d[points_in_canvas_mask]
            
            old_x_coord = np.insert(x_coord, 0, -1)
            old_y_coord = np.insert(y_coord, 0, -1)
            
            # Extract the screen coords (xy) as integers.
            new_x_coord = points_2d[:, 0].astype(np.int)
            new_y_coord = points_2d[:, 1].astype(np.int)

            x_coord = np.concatenate((new_x_coord, old_x_coord), axis=None)
            y_coord = np.concatenate((new_y_coord, old_y_coord), axis=None)

        return list(zip(x_coord, y_coord))


    def calculateYintersections(self, lane_list):
        """
        Transforms the 2D-image coordinates to the correct input format for the deep learning model, where only the y-values in steps of 10 are needed.
        This is done by the intersection of the line of the two points enclosing the y-value and the line f(x) = y. 
        For the lower half (0.6) of the image, if there are no points enclosing the y-value, the intersection is calculate by the first points existing.
        This may leads to inaccurate results at junctions, but completes the lines until the bottom of the image. 
        Since junctions are excluded from trainingsdata the incorrect results at junctions are as well.
        
        Args:
            lane_list: list. List of a lane, which contains its 2D-image-coordinates x, y.

        Returns:
            List of 2D-points in the correct format for the deep learning algorithm, where the elements of the list 
            are tuples. Each tuple contains an x and y value.
        """
        x_coord = []
        gap = False
        
        if(len(lane_list) > 2):
            last_point = lane_list[0]
            for xy_val in lane_list:
                if last_point == xy_val:
                    continue
                if xy_val[0]==-1:
                    gap = True
                    continue
                if last_point[0]==-1:
                    gap = True
                    last_point=[0.5* IMAGE_WIDTH, IMAGE_HEIGHT-1]
                for y_value in reversed(cfg.h_samples):
                    if gap and (last_point[1] >= y_value and xy_val[1] < y_value):
                        x_coord.append(-2)
                    elif (last_point == lane_list[0] and last_point[1] < y_value) and last_point[1] < IMAGE_HEIGHT - 0.4 * IMAGE_HEIGHT : # MODIFY
                        x_coord.append(-2)
                    elif (last_point[1] >= y_value and xy_val[1] < y_value) or (last_point == lane_list[0] and last_point[1] < y_value) and last_point[1] >= IMAGE_HEIGHT - 0.4 * IMAGE_HEIGHT:
                        if last_point[1]-xy_val[1] == 0:
                            intersection = last_point[1]
                        else:
                            intersection = xy_val[0] + ((y_value-xy_val[1])*(last_point[0]-xy_val[0]))/(last_point[1]-xy_val[1])
                            
                        if intersection >= IMAGE_WIDTH or intersection < 0:
                            x_coord.append(-2)
                        else:
                            x_coord.append(int(intersection))
                gap = False
                last_point = xy_val

            while len(x_coord) < len(cfg.h_samples):
                x_coord.append(-2)
            return list(zip(reversed(x_coord), cfg.h_samples))
        else:
            for i in cfg.h_samples:
                x_coord.append(-2)
            return list(zip(x_coord, cfg.h_samples))


    def filter2DLanepoints(self, lane_list, image):
        """
        Remove all calculated 2D-lanepoints from the lane_list, which are e.g. on a house or wall, with the help of the sematic segmentation camera.
  
        Args:
            lane_list: list. List of a lane, which contains its lanepoints. Needed to check, if a lanepoint is located on a semantic tag like road or roadline.
            image: numpy array. Semantic segmentation image providing specific colorlabels to identify, if road or roadline.

        Returns:
            filtered_lane_list: list. Every point, which is located on a road or roadline, but not on a building or wall.
        """
        filtered_lane_list = []
        for lanepoint in lane_list:
            x = lanepoint[0]
            y = lanepoint[1]
            if(np.any(image[y][x] == (128, 64, 128), axis=-1) or   # Road
               np.any(image[y][x] == (157, 234, 50), axis=-1) or   # Roadline
               np.any(image[y][x] == (244, 35, 232), axis=-1) or   # Sidewalk
               np.any(image[y][x] == (220, 220, 0), axis=-1) or    # Traffic sign
               np.any(image[y][x] == (0, 0, 142), axis=-1)):       # Vehicle
                filtered_lane_list.append(lanepoint)
            else:
                filtered_lane_list.append((-2, y))
                
        return filtered_lane_list



# ==============================================================================
# -- Main ---------------------------------------------------------------------
# ==============================================================================

def main():
    carlaGame = CarlaGame()
    carlaGame.execute()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
