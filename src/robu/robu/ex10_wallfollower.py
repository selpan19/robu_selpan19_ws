import math
import rclpy

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import LaserScan

from rclpy.qos import qos_profile_sensor_data

import numpy as np

from enum import IntEnum

class WallfollowerStates(IntEnum):
    WF_STATE_DETECTWALL = 0,
    WF_STATE_DRIVE2WALL = 1,
    WF_STATE_ROTATE2WALL =2,
    WF_STATE_FOLLOWWALL = 3

class WallFollower(rclpy.Node):
    def __init__(self):
        super().__init__('WallFollower')

def main(args=None):
    print("Hello from Wallfollower!")