#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

import os

from ament_index_python.packages import get_package_share_directory #zugriff auf fremdes Paket, gibt Speicherort

from launch import LaunchDescription #Beschreibung was gestastert werden soll

from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription

from launch_ros.actions import Node

from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
#Hilfsvariablen deklarieren
    tb3_world_dir = get_package_share_directory('turtlebot3_gazebo') 
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    robu_dir = get_package_share_directory('robu')

    map_yaml_file = os.path.join(robu_dir, 'maps', 'tb3_world', 'tb3_world.yaml')
    world_file = os.path.join(tb3_world_dir, 'worlds', 'turtlebot3_world.world')

    robot_name = 'turtlebot3_burger'

    robot_urdf = os.path.join(nav2_bringup_dir, 'urdf', robot_name+'.urdf')     #Roboter Beschreibung File für RVIZ
    robot_model_file = os.path.join(tb3_world_dir, 'models', robot_name+'model.sdf') #Roboter Beschreibung File für Gazebo
    
    robot_pose = {
        'x' : '-2.0',
        'y' : '-0.5',
        'z' : '0.01',
        'R' : '0.00',
        'P' : '0.00',
        'Y' : '0.00'
    }

    print("world: ", world_file)
    print("robot_urdf: ", robot_urdf)
    print("map_yaml_file: ", map_yaml_file)
    print("robot_model_file: ", robot_model_file)

    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_rviz_cmd = DeclareLaunchArgument('use_rviz', default_value=True)
    declare_headless_cmd = DeclareLaunchArgument('headless', default_value=False)
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value=True)

    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        cwd=[tb3_world_dir],
        output='screen'
    )

    start_gazebo_client_cmd = ExecuteProcess(
        condition = IfCondition(PythonExpression(['not', headless]))
        cmd=['gzclient'],
        cwd=[tb3_world_dir],
        output='screen'
    )


    #Roboter in Gazebo plazieren
    start_gazebo_spawner_cmd =  Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', robot_name
        '-file', robot_model_file,
        '-x', robot_pose['x'], '-y', robot_pose['y'], '-z', robot_pose['z'], 
        '-R', robot_pose['R'], '-P', robot_pose['P'], '-Y', robot_pose['Y']
        ]
    )

    #Beschreibung des Roboters für RVIZ einlesen
    with open(robot_urdf, 'r') as infp:
        robot_descr = infp.read()


