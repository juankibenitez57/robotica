import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class FolderImagePublisher(Node):
    def __init__(self):
        super().__init__('folder_image_publisher')
        self.declare_parameter('image_folder', '')
        folder = self.get_parameter('image_folder').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)

        extensions = ('.jpg', '.jpeg', '.png', '.bmp')
        self.images = sorted([
            os.path.join(folder, f) for f in os.listdir(folder)
            if f.lower().endswith(extensions)
        ])
        self.index = 0

        if not self.images:
            self.get_logger().error(f'No se encontraron imágenes en: {folder}')
            return

        self.get_logger().info(f'Publicando {len(self.images)} imagen(es) desde: {folder}')
        self.timer = self.create_timer(1.0, self.publish_image)

    def publish_image(self):
        path = self.images[self.index]
        cv_image = cv2.imread(path)
        if cv_image is None:
            self.get_logger().warn(f'No se pudo leer: {path}')
            return
        cv_image = cv2.resize(cv_image, (640, 480))
        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicada: {os.path.basename(path)}')
        self.index = (self.index + 1) % len(self.images)


def main(args=None):
    rclpy.init(args=args)
    node = FolderImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
