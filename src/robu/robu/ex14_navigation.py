#CLI - commands
#1. action list: ros2 action list
#2. action info: ros2 action info <action_name> -t (-t für Datentyp)
#3. interface list: ros2 interface show <interface_name>
#4. action call: ros2 action send_goal \navigateToPose 

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from std_msgs.msg import String

class Nav2Pos(Node):
    def __init__(self):
        super().__init__('nav2pose')
        self.pose_subscriber = self.create_subscription(String, '/nav2pose', self.nav2pose_callback, 10)

        
        self.declare_parameters('', 
        [('pose_initial', [0.0, 0.0, 0.0, 1.0]),
        ('pose_a', [3.75, 1.0, 0.0, 1.0])]), #x, y, z, w

        self.navigator = BasicNavigator() #creates a navigator object
        #set initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frage_id = 'map'
        initial_pose.pose.position.x = self.get_parameter('pose_initial').value[0]
        initial_pose.pose.position.y = self.get_parameter('pose_initial').value[1]
        initial_pose.pose.orientation.z = self.get_parameter('pose_initial').value[2]
        initial_pose.pose.orientation.w = self.get_parameter('pose_initial').value[3]
        

