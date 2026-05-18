import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class MoveRobot(Node):

    def __init__(self):
        super().__init__('move_robot')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.run)
        self.start_time = self.get_clock().now()
        self.state = 'forward'
        self.get_logger().info('Iniciando secuencia: adelante 1s → izquierda')

    def run(self):
        msg = Twist()
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if self.state == 'forward':
            if elapsed < 1.0:
                msg.linear.x = 0.5
                self.get_logger().info(f'Adelante ({elapsed:.1f}s)')
            else:
                self.state = 'left'
                self.start_time = self.get_clock().now()

        elif self.state == 'left':
            if elapsed < 2.0:
                msg.angular.z = 0.8
                self.get_logger().info(f'Izquierda ({elapsed:.1f}s)')
            else:
                self.state = 'stop'
                self.get_logger().info('Secuencia completada.')

        elif self.state == 'stop':
            self.timer.cancel()

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
