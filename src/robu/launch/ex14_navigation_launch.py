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

    
