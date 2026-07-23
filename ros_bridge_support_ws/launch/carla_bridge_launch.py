import launch
from launch import LaunchService
from launch.actions import RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

from ament_index_python.packages import get_package_share_directory
import os
import subprocess
from pathlib import Path
from parser_launch import inject_launch_arguments, get_ros_parser

ROS_WS = str(Path(__file__).resolve().parents[1])
SDK_WS = str(Path(__file__).resolve().parents[2])
USER_NAME = os.environ.get('USER')

def get_git_info():
    """Get git version information"""
    try:
        git_version = subprocess.check_output(
            ['git', 'describe', '--tags', '--always', '--dirty'],
            cwd=SDK_WS,
            stderr=subprocess.DEVNULL,
            text=True
        ).strip()
    except:
        git_version = "unknown"
    
    try:
        git_hash = subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'],
            cwd=SDK_WS,
            stderr=subprocess.DEVNULL,
            text=True
        ).strip()
    except:
        git_hash = "unknown"
    
    try:
        git_branch = subprocess.check_output(
            ['git', 'rev-parse', '--abbrev-ref', 'HEAD'],
            cwd=SDK_WS,
            stderr=subprocess.DEVNULL,
            text=True
        ).strip()
    except:
        git_branch = "unknown"
    
    return git_version, git_hash, git_branch

def define_launch_argument():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost',
            description='IP of the CARLA server'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000',
            description='TCP port of the CARLA server'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='20',
            description='Time to wait for a successful connection to the CARLA server'
        ),
        launch.actions.DeclareLaunchArgument(
            name='passive',
            default_value='False',
            description='When enabled, the ROS bridge will take a backseat and another client must tick the world (only in synchronous mode)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode',
            default_value='True',
            description='Enable/disable synchronous mode. If enabled, the ROS bridge waits until the expected data is received for all sensors'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False',
            description='When enabled, pauses the tick until a vehicle control is completed (only in synchronous mode)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05',
            description='Simulation time (delta seconds) between simulation steps'
        ),
        launch.actions.DeclareLaunchArgument(
            name='town',
            default_value='Town04_Opt',
            description='Either use an available CARLA town (eg. "Town04") or an OpenDRIVE file (ending in .xodr)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='register_all_sensors',
            default_value='False',
            description='Enable/disable the registration of all sensors. If disabled, only sensors spawned by the bridge are registered'
        ),
        launch.actions.DeclareLaunchArgument(
            name='ego_vehicle_role_name',
            default_value=["hero", "ego_vehicle", "hero0", "hero1", "hero2",
                           "hero3", "hero4", "hero5", "hero6", "hero7", "hero8", "hero9"],
            description='Role names to identify ego vehicles. Relevant topics will be created so these vehicles will be able to be controlled from ROS.'
        ),
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value=os.path.join(SDK_WS, 'common', 'config', 'carla', 'vehicle_1.json'),
            description='Path to definition object file (format file is JSON)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_point_ego_vehicle',
            default_value='None',
            description='If None, the vehicle will be spawned at a random location'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_sensors_only',
            default_value='False',
            description='If True, this will check if an actor with the same id and type as the one specified in the .json file is already active and if so, attach the sensors to this actor.'
        )
    ])
    return ld

def define_main_action():
    ld = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ros_bridge'), 'carla_ros_bridge.launch.py')
            ),
            launch_arguments = {
                'host': launch.substitutions.LaunchConfiguration('host'),
                'port': launch.substitutions.LaunchConfiguration('port'),
                'timeout': launch.substitutions.LaunchConfiguration('timeout'),
                'passive': launch.substitutions.LaunchConfiguration('passive'),
                'synchronous_mode': launch.substitutions.LaunchConfiguration('synchronous_mode'),
                'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command'),
                'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds'),
                'town': launch.substitutions.LaunchConfiguration('town'),
                'register_all_sensors': launch.substitutions.LaunchConfiguration('register_all_sensors'),
                'ego_vehicle_role_name': launch.substitutions.LaunchConfiguration('ego_vehicle_role_name')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_spawn_objects'), 'carla_spawn_objects.launch.py')
            ),
            launch_arguments = {
                'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file'),
                'spawn_point_ego_vehicle': launch.substitutions.LaunchConfiguration('spawn_point_ego_vehicle'),
                'spawn_sensors_only': launch.substitutions.LaunchConfiguration('spawn_sensors_only')
            }.items()
        ),
        launch.actions.ExecuteProcess(
            cmd=[
                'python',
                os.path.join(ROS_WS, 'src', 'adas_bridge', 'scripts', 'wheel_sensor.py')
            ],
            output='log'
        ),
        # launch.actions.ExecuteProcess(
        #     cmd=[
        #         'python',
        #         os.path.join(ROS_WS, 'src', 'adas_bridge', 'scripts', 'carla_api.py')
        #     ],
        #     output='log'
        # ),
        launch.actions.ExecuteProcess(
            cmd=[
                'python',
                os.path.join(SDK_WS, 'script', 'carla_script', 'manual_control_ego_vehicle.py'),
                '--uid', '0'
            ],
            output='log'
        )
    ])
    return ld

def generate_launch_description():
    ld = launch.LaunchDescription([
        *define_launch_argument().entities,
        *define_main_action().entities,
        RegisterEventHandler(
            OnProcessExit(
                on_exit=[
                    LogInfo(msg="A node has exited. Shutting down entire launch..."),
                    EmitEvent(event=Shutdown())
                ]
            )   
        )
    ])
    return ld

def main():
    # Display version information
    git_version, git_hash, git_branch = get_git_info()
    print("==============================================")
    print(f"BV CARLA Bridge {git_version}")
    print(f"Commit: {git_hash}")
    print(f"Branch: {git_branch}")
    print("==============================================")

    parser = get_ros_parser()
    args = parser.parse_args()

    # Create a LaunchDescription with the node
    ld = generate_launch_description()

    # Start the launch service
    ls = LaunchService()
    ls.include_launch_description(ld)
    inject_launch_arguments(ls, args.ros_args)
    ls.run()

if __name__ == '__main__':
    main()
