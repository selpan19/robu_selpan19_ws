#https://docs.ros.org/en/foxy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

#Importiert Schnittstelle
from robu_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        #Actionserver erstellen
        self._action_server = ActionServer(
            #Klasse in der der Actionsserver erstellt werden soll
            self,
            #Schnittstelle
            Fibonacci,
            #Bezeichnung für action
            'fibonacci',
            #Methode, die aufgerufen wird wenn Anfrage kommt
            self.execute_callback)

    #Anfragemethode anlegen (self, ziel): This is the method that will be called to execute a goal once it is accepted.
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
          sequence.append(sequence[i] + sequence[i-1])

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = sequence
        return result
    


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    #wartet bis etwas passiert
    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()