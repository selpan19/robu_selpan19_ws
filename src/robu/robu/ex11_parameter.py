import rclpy
from rclpy.node import Node

class MinimalParamter(Node):
    def __init__(self):
        super().__init__('MinimalParamter')

        self.declare_parameter('my_parameter')

        my_param = self.get_parameter('my_parameter')

        print("my_parameter: ", my_param)


def main():
    rclpy.init()

    node = MinimalParamter()

    rclpy.spin(node)
    