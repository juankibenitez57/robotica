import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TextPublisher(Node):
    def __init__(self):
        super().__init__('text_publisher')
        self.publisher_ = self.create_publisher(String, 'texto_usuario', 10)

    def publish_text(self):
        while rclpy.ok():
            text = input("Introduce un texto para analizar sentimiento: ")

            msg = String()
            msg.data = text

            self.publisher_.publish(msg)
            self.get_logger().info(f'Texto publicado: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    node = TextPublisher()

    try:
        node.publish_text()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()