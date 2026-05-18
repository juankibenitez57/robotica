import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.current_image = None
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        self.get_logger().info('Suscriptor iniciado, esperando imágenes...')

    def listener_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return
        self.get_logger().info(f'Imagen recibida: {self.current_image.shape[1]}x{self.current_image.shape[0]}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.current_image is not None:
            cv2.imshow('Camera Subscriber', node.current_image)
        cv2.waitKey(1)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
