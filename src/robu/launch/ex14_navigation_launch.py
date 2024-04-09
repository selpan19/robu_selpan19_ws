import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,      #Zum Deklarieren von Argumenten beim Starten über die Kommandozeile
    ExecuteProcess,             #Zum Ausführen von Prozessen
    IncludeLaunchDescription,   #Zum Einbinden von anderen Launch-Dateien
    SetEnvironmentVariable      #Zum Setzen von Umgebungsvariablen
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    tb3_world_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    robu_dir = get_package_share_directory('robu')

    map_yaml_file = os.path.join(robu_dir, 'maps', 'tb3_world', 'tb3_world.yaml')
    world = os.path.join(tb3_world_dir, 'worlds', 'turtlebot3_world.world')
    robot_name = 'turtlebot3_burger'
    robot_urdf = os.path.join(nav2_bringup_dir, 'urdf', robot_name + '.urdf')
    robot_model_file = os.path.join(tb3_world_dir, 'models', robot_name, 'model.sdf')

    robot_pose = {'x': '-2.00',
                  'y': '-0.50',
                  'z': '0.01',
                  'R': '0.00',
                  'P': '0.00',
                  'Y': '0.00'}
    
    print("world: ", world)
    print("robot_urdf: ", robot_urdf)
    print("map_yaml_file: ", map_yaml_file)
    print("robot_model_file: ", robot_model_file)

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    # Launch configuration variables
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    envar_cmd = SetEnvironmentVariable('ROS_DOMAIN_ID', '100')
                           
    # Declare the launch arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RVIZ'
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless', default_value='False', description='Whether to execute gzclient)'
    )

    # start the simulation
    start_gazebo_server_cmd = ExecuteProcess(
        #cmd=['gzserver', '-s', 'libgazebo_ros_factory.so', world],
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
        cwd=[tb3_world_dir],
        output='screen',
    )

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(['not ', headless])),
        cmd=['gzclient'],
        cwd=[tb3_world_dir],
        output='screen',
    )

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', robot_model_file,
            '-x', robot_pose['x'], '-y', robot_pose['y'], '-z', robot_pose['z'],
            '-R', robot_pose['R'], '-P', robot_pose['P'], '-Y', robot_pose['Y']])
    

    with open(robot_urdf, 'r') as infp:
        robot_desc = infp.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        },
        ],
        # remappings=remappings
    )

    # start the visualization
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': '', 'use_namespace': 'False'}.items(),
    )

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={'map': map_yaml_file}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(envar_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    
    return ld