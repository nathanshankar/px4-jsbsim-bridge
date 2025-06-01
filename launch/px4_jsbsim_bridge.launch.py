import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from lxml import etree
import os

RENDERING_XML = """
<rendering>
  <camera-group>
    <window>
      <name>main</name>
      <display>0</display>
      <screen>0</screen>
      <width>1920</width>
      <height>1080</height>
      <x>0</x>
      <y>0</y>
    </window>
    <camera>
      <window><name>main</name></window>
      <view><heading-deg>0</heading-deg><x>1.0</x><y>0.0</y><z>0.0</z><pitch-deg>0</pitch-deg></view>
      <viewport><x>0</x><y>0</y><width>960</width><height>1080</height></viewport>
    </camera>
    <camera>
      <window><name>main</name></window>
      <view><heading-deg>0</heading-deg><x>0.0</x><y>0.0</y><z>0.0</z><pitch-deg>0</pitch-deg></view>
      <viewport><x>960</x><y>0</y><width>960</width><height>1080</height></viewport>
    </camera>
    <gui><window><name>main</name></window></gui>
  </camera-group>
</rendering>
"""

def inject_rendering_block(models_dir: str, vehicle: str):
    vehicle_to_model = {
        'jsbsim_rascal': 'Rascal',
        'jsbsim_quadrotor_x': 'quadrotor_x',
        'malolo': 'ATI-Resolution',
        'quadrotor_x': 'quadrotor_x',
        'hexarotor_x': 'hexarotor_x'
    }
    
    model_name = vehicle_to_model.get(vehicle)
    if not model_name:
        print(f"[!] Vehicle '{vehicle}' not found in vehicle_to_model map. Skipping rendering injection.")
        return
    
    model_dir = os.path.join(models_dir, model_name)
    if not os.path.isdir(model_dir):
        print(f"[!] Model directory {model_dir} does not exist. Skipping rendering injection.")
        return
    
    # Find all '-set.xml' files inside the model directory
    set_files = [f for f in os.listdir(model_dir) if f.endswith('-set.xml')]
    if not set_files:
        print(f"[!] No '-set.xml' files found in {model_dir}. Skipping rendering injection.")
        return
    
    for set_file_name in set_files:
        set_file = os.path.join(model_dir, set_file_name)
        try:
            parser = etree.XMLParser(remove_blank_text=True)
            tree = etree.parse(set_file, parser)
            root = tree.getroot()
            sim_nodes = root.xpath('/PropertyList/sim')
            sim_node = sim_nodes[0] if sim_nodes else None

            if sim_node is None:
                print(f"[!] No <sim> node found in {set_file}, skipping.")
                continue
            
            # Remove all <view> nodes outside <rendering> inside <sim>
            old_views = sim_node.findall('view')
            if old_views:
                for v in old_views:
                    sim_node.remove(v)
                print(f"[i] Removed {len(old_views)} standalone <view> nodes from {set_file}")
            
            # Remove existing <rendering> if any, to avoid duplicates
            old_rendering = sim_node.find('rendering')
            if old_rendering is not None:
                sim_node.remove(old_rendering)
                print(f"[i] Removed existing <rendering> from {set_file}")

            # Inject our split view rendering XML
            rendering_element = etree.fromstring(RENDERING_XML)
            sim_node.append(rendering_element)
            tree.write(set_file, pretty_print=True, xml_declaration=True, encoding="UTF-8")
            print(f"[✓] Injected rendering into {set_file}")
        except Exception as e:
            print(f"[!] Error processing {set_file}: {e}")


def generate_launch_description():

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

    set_px4_sim_model = SetEnvironmentVariable(
        name='PX4_SIM_MODEL', value=LaunchConfiguration('vehicle'))
    set_px4_estimator = SetEnvironmentVariable(
        name='PX4_ESTIMATOR', value=LaunchConfiguration('est'))

    mavros_launch_dir = '/opt/ros/jazzy/share/mavros/launch'
    px4_config_dir = '/home/nathan/jsbsim_ros2/src/px4-jsbsim-bridge/configs'

    # Include mavros node.launch.py with args
    mavros_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(mavros_launch_dir, 'node.launch')),
        launch_arguments={
            'pluginlists_yaml': os.path.join(mavros_launch_dir, 'px4_pluginlists.yaml'),
            'config_yaml': os.path.join(px4_config_dir, 'px4_config.yaml'),
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'tgt_system': LaunchConfiguration('tgt_system'),
            'tgt_component': LaunchConfiguration('tgt_component'),
            'log_output': LaunchConfiguration('log_output'),
            'fcu_protocol': LaunchConfiguration('fcu_protocol'),
            'respawn_mavros': LaunchConfiguration('respawn_mavros'),
            'use_sim_time': 'true',
        }.items()
    )

    # PX4 SITL interactive (fixed startup script path and cwd)
    px4_sitl_interactive = ExecuteProcess(
        cmd=[
            os.path.join(os.getenv('HOME'), 'PX4-Autopilot', 'build', 'px4_sitl_default', 'bin', 'px4'),
            '.',
            '-s',
            'init.d-posix/rcS'  # no leading dot
        ],
        cwd=os.path.join(os.getenv('HOME'), 'PX4-Autopilot', 'etc'),  # correct cwd for PX4 scripts
        output='screen',
        condition=IfCondition(LaunchConfiguration('interactive'))
    )

    # PX4 SITL noninteractive (fix condition to UnlessCondition)
    px4_sitl_noninteractive = ExecuteProcess(
        cmd=[
            os.path.join(os.getenv('HOME'), 'PX4-Autopilot', 'build', 'px4_sitl_default', 'bin', 'px4'),
            '.',
            '-s',
            'init.d-posix/rcS',
            '-d',
        ],
        cwd=os.path.join(os.getenv('HOME'), 'PX4-Autopilot', 'etc'),  # consistent cwd
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('interactive'))  # launch only if not interactive
    )

    def run_injection(context, *args, **kwargs):
        models_dir = os.path.join(os.getenv('HOME'), 'jsbsim_ros2', 'src', 'px4-jsbsim-bridge', 'models')
        vehicle = LaunchConfiguration('vehicle').perform(context)
        inject_rendering_block(models_dir, vehicle)
        return []


    inject_rendering = OpaqueFunction(function=run_injection)


    # FlightGear process with dynamic aircraft and model path
    # http://localhost:5500/screenshot?stream=0
    def launch_flightgear(context, *args, **kwargs):
        vehicle = LaunchConfiguration('vehicle').perform(context)
        vehicle_to_model = {
            'jsbsim_rascal': 'Rascal110-JSBSim',
            'jsbsim_quadrotor_x': 'quadrotor_x',
            'malolo': 'Malolo1',
            'quadrotor_x': 'quadrotor_x',
            'hexarotor_x': 'hexarotor_x'
        }
        aircraft_model = vehicle_to_model.get(vehicle, 'quadrotor_x')

        return [ExecuteProcess(
            cmd=[
                'fgfs',
                '--fdm=null',
                '--native-fdm=socket,in,60,,5550,udp',
                '--aircraft=' + aircraft_model,
                '--fg-aircraft=' + os.path.join(os.getenv('HOME'), 'jsbsim_ros2', 'src', 'px4-jsbsim-bridge', 'models'),
                '--airport=LSZH',
                '--disable-hud',
                '--disable-ai-models',
                '--prop:/sim/rendering/osg-displaysettings/stereo-mode=OFF',
                '--httpd=5500',
                '--enable-fullscreen',
            ],
            output='screen'
        )]

    flightgear_process = OpaqueFunction(function=launch_flightgear)


    jsbsim_stream = Node(
        package='jsbsim_bridge',
        executable='jsbsim_stream',
        name='jsbsim_stream',
        output='screen'
    )

    # JSBSim bridge node with vehicle argument
    jsbsim_bridge_node = Node(
        package='jsbsim_bridge',
        executable='jsbsim_bridge_node',
        name='jsbsim_bridge',
        output='screen',
        arguments=[LaunchConfiguration('vehicle')]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join('/home/nathan/jsbsim_ros2/src/px4-jsbsim-bridge/rviz/slam_view.rviz')],
        condition=IfCondition(LaunchConfiguration('visualization'))
    )

    return LaunchDescription([
        est_arg,
        vehicle_arg,
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        command_input_arg,
        gazebo_simulation_arg,
        visualization_arg,
        log_output_arg,
        fcu_protocol_arg,
        respawn_mavros_arg,
        interactive_arg,

        set_px4_sim_model,
        set_px4_estimator,
        inject_rendering,

        mavros_launch,
        px4_sitl_interactive,
        px4_sitl_noninteractive,
        jsbsim_bridge_node,
        flightgear_process,
        jsbsim_stream,
        rviz2_node,
    ])
