"""
nlp/intent_node.py — Clasificador semántico de intención
---------------------------------------------------------

Recibe texto libre en /nlp/input y publica un CognitiveCommand
con la acción e intención detectadas mediante zero-shot classification.

Modelo por defecto: cross-encoder/nli-distilroberta-base (~166 MB)
Primera ejecución descarga el modelo de Hugging Face automáticamente.

Topics:
  Sub:  /nlp/input          (std_msgs/String)
  Pub:  /cognitive/command  (tb3_msgs/CognitiveCommand)

Parámetros ROS2:
  model  (string)  — modelo HuggingFace a usar
  device (int)     — -1 CPU, 0 GPU

Ejemplos de input → output:
  "ve a la cocina"          → {action: navigate,  target: kitchen}
  "busca una botella"       → {action: search,    target: bottle}
  "explora la habitación"   → {action: explore,   target: room}
  "acércate a la silla"     → {action: approach,  target: chair}
  "para"                    → {action: stop,      target: }
  "qué ves ahora mismo"     → {action: report,    target: }
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tb3_msgs.msg import CognitiveCommand


# ── Etiquetas semánticas para zero-shot classification ────────────────────────
# Descripciones largas → mejor discriminación del modelo NLI
INTENT_LABELS = [
    "navigate to a specific room or location in the house",
    "search for a specific object or item",
    "explore and map an unknown area or environment",
    "approach or move closer to a detected target",
    "stop all movement and stand still",
    "report status, describe what is visible, or answer a question",
]

ACTION_MAP = {
    "navigate to a specific room or location in the house": "navigate",
    "search for a specific object or item":                  "search",
    "explore and map an unknown area or environment":        "explore",
    "approach or move closer to a detected target":          "approach",
    "stop all movement and stand still":                     "stop",
    "report status, describe what is visible, or answer a question": "report",
}

# ── Vocabulario de entidades conocidas ────────────────────────────────────────
# Fase 1: extracción simple post-clasificación.
# Fase 2 (LangChain): reemplazado por NER real.
LOCATIONS = [
    "kitchen", "living room", "bedroom", "bathroom", "hallway",
    "corridor", "garage", "office", "cocina", "habitacion",
    "salon", "bano", "pasillo", "entrada", "comedor",
]

OBJECTS = [
    "bottle", "chair", "person", "table", "cup", "phone",
    "book", "bag", "box", "ball", "laptop", "plant",
    "botella", "silla", "persona", "mesa", "taza",
    "libro", "bolsa", "caja", "balon", "planta",
]


class NLPIntentNode(Node):

    def __init__(self):
        super().__init__('nlp_intent_node')

        model_name = self.declare_parameter(
            'model', 'cross-encoder/nli-distilroberta-base').get_parameter_value().string_value
        device = self.declare_parameter(
            'device', -1).get_parameter_value().integer_value

        self.get_logger().info(f'Cargando modelo NLP: {model_name}')
        self.get_logger().info('Primera ejecución descarga ~166 MB (solo una vez)...')

        # Import aquí para no bloquear el arranque si transformers no está
        try:
            from transformers import pipeline as hf_pipeline
            self._classifier = hf_pipeline(
                'zero-shot-classification',
                model=model_name,
                device=device,
            )
        except ImportError:
            self.get_logger().fatal(
                'transformers no instalado. Ejecuta: pip install transformers torch')
            raise

        self.get_logger().info('Modelo NLP listo.')

        self._sub = self.create_subscription(
            String, '/nlp/input', self._text_callback, 10)
        self._pub = self.create_publisher(
            CognitiveCommand, '/cognitive/command', 10)

        self.get_logger().info(
            'NLPIntentNode activo.\n'
            '  Sub: /nlp/input\n'
            '  Pub: /cognitive/command')

    # ── Callback principal ────────────────────────────────────────────────────

    def _text_callback(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f'Input recibido: "{text}"')

        result = self._classifier(text, INTENT_LABELS, multi_label=False)

        top_label = result['labels'][0]
        top_score = float(result['scores'][0])
        action    = ACTION_MAP[top_label]
        target    = self._extract_target(text, action)

        cmd = CognitiveCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.action     = action
        cmd.target     = target
        cmd.confidence = top_score
        cmd.raw_text   = text

        self._pub.publish(cmd)

        self.get_logger().info(
            f'Comando publicado → '
            f'action={action!r}  target={target!r}  conf={top_score:.2f}')

    # ── Extracción de entidad destino ─────────────────────────────────────────
    # Encuentra la primera entidad conocida mencionada en el texto.
    # Reemplazado por NER real en Fase 2 (LangChain tools).

    def _extract_target(self, text: str, action: str) -> str:
        t = text.lower()

        if action in ('navigate', 'explore'):
            for loc in LOCATIONS:
                if loc in t:
                    return loc

        if action in ('search', 'approach'):
            for obj in OBJECTS:
                if obj in t:
                    return obj

        # Fallback: cualquier entidad en cualquier acción
        for loc in LOCATIONS:
            if loc in t:
                return loc
        for obj in OBJECTS:
            if obj in t:
                return obj

        return ''


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = NLPIntentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
