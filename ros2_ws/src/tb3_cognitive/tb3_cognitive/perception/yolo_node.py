"""
perception/yolo_node.py — Detector YOLOv8 para búsqueda visual de objetos (Fase 5)

Sub:  /camera/image_raw   (sensor_msgs/Image)
Pub:  /detected_objects   (tb3_msgs/DetectedObject)  — una por cada detección

Parámetros:
  model       — ruta o nombre del modelo YOLOv8 (default: yolov8n.pt)
  confidence  — umbral mínimo de detección     (default: 0.50)
  device      — CPU=-1, GPU=0,1,...            (default: -1)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tb3_msgs.msg import DetectedObject

from ultralytics import YOLO


class YoloDetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector_node')

        model_name = self.declare_parameter(
            'model', 'yolov8n.pt').get_parameter_value().string_value
        self._conf = self.declare_parameter(
            'confidence', 0.50).get_parameter_value().double_value
        device_idx = self.declare_parameter(
            'device', -1).get_parameter_value().integer_value

        device = 'cpu' if device_idx < 0 else str(device_idx)
        self.get_logger().info(
            f'[YOLO] Cargando modelo: {model_name}  device={device}')
        self._model = YOLO(model_name)
        self._model.to(device)

        self._pub = self.create_publisher(DetectedObject, '/detected_objects', 10)
        # queue=1: siempre el fotograma más reciente, descarta los anteriores
        self._sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, 1)

        self.get_logger().info(
            f'[YOLO] Activo — conf≥{self._conf:.2f}  '
            'Sub: /camera/image_raw  Pub: /detected_objects')

    # ── Callback de imagen ────────────────────────────────────────────────────

    def _image_cb(self, msg: Image) -> None:
        if msg.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
            self.get_logger().warn_once(
                f'[YOLO] Encoding no soportado: {msg.encoding!r}')
            return

        channels = 4 if msg.encoding in ('rgba8', 'bgra8') else 3
        img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
            msg.height, msg.width, channels)

        # Normaliza a BGR para ultralytics (entrenado en imágenes OpenCV)
        if msg.encoding == 'rgb8':
            img = img[:, :, ::-1].copy()
        elif msg.encoding == 'rgba8':
            img = img[:, :, :3][:, :, ::-1].copy()
        elif msg.encoding == 'bgra8':
            img = img[:, :, :3].copy()

        results = self._model(img, conf=self._conf, verbose=False)

        h, w = msg.height, msg.width
        now  = self.get_clock().now().to_msg()

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                conf     = float(box.conf[0])
                label    = self._model.names[int(box.cls[0])]

                cx   = (x1 + x2) / 2.0 / w
                cy   = (y1 + y2) / 2.0 / h
                area = float((x2 - x1) * (y2 - y1)) / (w * h)

                det = DetectedObject()
                det.header.stamp    = now
                det.header.frame_id = 'camera_link'
                det.label           = label
                det.confidence      = conf
                det.bbox_x1         = x1
                det.bbox_y1         = y1
                det.bbox_x2         = x2
                det.bbox_y2         = y2
                det.center_x_norm   = float(cx)
                det.center_y_norm   = float(cy)
                det.area_norm       = float(area)

                self._pub.publish(det)
                self.get_logger().debug(
                    f'[YOLO] {label}  conf={conf:.2f}  '
                    f'cx={cx:.2f}  area={area:.3f}')


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
