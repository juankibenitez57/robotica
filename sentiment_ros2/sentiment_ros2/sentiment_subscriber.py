import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from transformers import pipeline


class SentimentSubscriber(Node):
    def __init__(self):
        super().__init__('sentiment_subscriber')

        self.subscription = self.create_subscription(
            String,
            'texto_usuario',
            self.listener_callback,
            10
        )

        self.classifier = pipeline(
            "sentiment-analysis",
            model="nlptown/bert-base-multilingual-uncased-sentiment"
        )

        self.get_logger().info("Nodo de análisis de sentimiento iniciado.")

    def listener_callback(self, msg):
        text = msg.data

        result = self.classifier(text)[0]

        label = result["label"]
        score = result["score"]

        self.get_logger().info(
            f'Texto recibido: "{text}" | Sentimiento: {label} | Confianza: {score:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = SentimentSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()