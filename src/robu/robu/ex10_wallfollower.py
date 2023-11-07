import math
import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from rclpy.qos import qos_profile_sensor_data

import numpy as np

from enum import IntEnum

ROBOT_DIRECTION_FRONT_INDEX = 0
ROBOT_DIRECTION_RIGHTFRONT_INDEX = 45
ROBOT_DIRECTION_RIGHT_INDEX = 90
ROBOT_DIRECTION_RIGHTREAR_INDEX = 135
ROBOT_DIRECTION_REAR_INDEX = 180
ROBOT_DIRECTION_LEFTREAR_INDEX = 225
ROBOT_DIRECTION_LEFT_INDEX = 270
ROBOT_DIRECTION_LEFTFRONT_INDEX = 315


class WallfollowerStates(IntEnum):
    WF_STATE_DETECTWALL = 0,
    WF_STATE_DRIVE2WALL = 1,
    WF_STATE_ROTATE2WALL =2,
    WF_STATE_FOLLOWWALL = 3

class WallFollower(Node):
    def __init__(self):
        super().__init__('WallFollower')
        self.scan_subscriber = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data)
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", qos_profile_sensor_data)

        self.left_dist = 9999999.9 # Initialisiere den Varible auf einen ungueltigen Wert
        self.leftfront_dist = 9999999.9
        self.front_dist = 9999999.9
        self.rightfront_dist = 9999999.9
        self.right_dist = 9999999.9
        self.rear_dist = 9999999.9
        self.distances = []

        self.wallfollower_state = WallfollowerStates.WF_STATE_DETECTWALL

        self.forward_speed_wf_slow = 0.05
        self.forward_speed_wf_fast = 0.1

        self.turning_speed_wf_slow = 0.1
        self.turning_speed_wf_fast = 1.0

        self.dist_thresh_wf = 0.3
        self.dist_hysteresis_wf = 0.02

        self.dist_laser_offset = 0.03

        self.valid_lidar_data = False
        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        if self.valid_lidar_data:            #Beim erstmaligen Aufruf liegen noch keine LidarDaten vor
            self.follow_wall()
        
    def follow_wall(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        if self.wallfollower_state == WallfollowerStates.WF_STATE_DETECTWALL:
            dist_min = min(self.distances)
            #Drehe Roboter solange bis auf der X-Achse der Mindestabstand erscheint
            if(self.front_dist > (dist_min + self.dist_laser_offset)):
                #Wenn der Roboter kurz vor dem Mindestabstand ist, verringert dieser die Drehgeschwindigkeit
                if abs(self.front_dist - dist_min) < 0.2:
                    msg.angular.z = self.turning_speed_wf_slow
                else: 
                    msg.angular.z = self.turning_speed_wf_fast
            else:
                print("WF_STATE_DRIVE2WALL")
                self.wallfollower_state = WallfollowerStates.WF_STATE_DRIVE2WALL


    def scan_callback(self, msg):
        
        self.left_dist = msg.ranges[ROBOT_DIRECTION_LEFT_INDEX]
        self.leftfront_dist = msg.ranges[ROBOT_DIRECTION_LEFTFRONT_INDEX]
        self.front_dist = msg.ranges[ROBOT_DIRECTION_FRONT_INDEX]
        self.rightfront_dist = msg.ranges[ROBOT_DIRECTION_RIGHTFRONT_INDEX]
        self.right_dist = msg.ranges[ROBOT_DIRECTION_RIGHT_INDEX]
        self.rear_dist = msg.ranges[ROBOT_DIRECTION_REAR_INDEX]
        self.distances = msg.ranges

        print("ld: %.2f m" % self.left_dist,
              "lfd: %.2f m" % self.leftfront_dist,
              "rd: %.2f m" % self.right_dist,
              "rfd: %.2f m" % self.rightfront_dist,
              "fd: %.2f m" % self.front_dist,
              "rd: %.2f m" % self.rear_dist
              )

def main(args=None):
    rclpy.init(args = args)
    wallfollower = WallFollower()


    rclpy.spin(wallfollower)

    wallfollower.destroy_node()
    rclpy.shutdown()