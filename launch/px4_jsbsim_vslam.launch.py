from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    # Declare launch arguments with defaults
    est_arg = DeclareLaunchArgument('est', default_value='ekf2')
    vehicle_arg = DeclareLaunchArgument('vehicle', default_value='jsbsim_quadrotor_x')
    fcu_url_arg = DeclareLaunchArgument('fcu_url', default_value='udp://:14540@127.0.0.1:14557')
    gcs_url_arg = DeclareLaunchArgument('gcs_url', default_value='')
    tgt_system_arg = DeclareLaunchArgument('tgt_system', default_value='1')
    tgt_component_arg = DeclareLaunchArgument('tgt_component', default_value='1')
    command_input_arg = DeclareLaunchArgument('command_input', default_value='2')
    gazebo_simulation_arg = DeclareLaunchArgument('gazebo_simulation', default_value='true')
    visualization_arg = DeclareLaunchArgument('visualization', default_value='true')
    log_output_arg = DeclareLaunchArgument('log_output', default_value='screen')
    fcu_protocol_arg = DeclareLaunchArgument('fcu_protocol', default_value='v2.0')
    respawn_mavros_arg = DeclareLaunchArgument('respawn_mavros', default_value='false')
    interactive_arg = DeclareLaunchArgument('interactive', default_value='true')

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('jsbsim_bridge'),
            '/launch',
            '/px4_jsbsim_bridge.launch.py'
        ]),
        launch_arguments={
            'est': LaunchConfiguration('est'),
            'vehicle': LaunchConfiguration('vehicle'),
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'tgt_system': LaunchConfiguration('tgt_system'),
            'tgt_component': LaunchConfiguration('tgt_component'),
            'command_input': LaunchConfiguration('command_input'),
            'gazebo_simulation': LaunchConfiguration('gazebo_simulation'),
            'visualization': LaunchConfiguration('visualization'),
            'log_output': LaunchConfiguration('log_output'),
            'fcu_protocol': LaunchConfiguration('fcu_protocol'),
            'respawn_mavros': LaunchConfiguration('respawn_mavros'),
            'interactive': LaunchConfiguration('interactive'),
        }.items()
    )



    ld.add_action(est_arg)
    ld.add_action(vehicle_arg)
    ld.add_action(fcu_url_arg)
    ld.add_action(gcs_url_arg)
    ld.add_action(tgt_system_arg)
    ld.add_action(tgt_component_arg)
    ld.add_action(command_input_arg)
    ld.add_action(gazebo_simulation_arg)
    ld.add_action(visualization_arg)
    ld.add_action(log_output_arg)
    ld.add_action(fcu_protocol_arg)
    ld.add_action(respawn_mavros_arg)
    ld.add_action(interactive_arg)
    ld.add_action(bridge_launch)
    # ld.add_action(cam_sync)
    # ld.add_action(pointcloud_startup)

    return ld