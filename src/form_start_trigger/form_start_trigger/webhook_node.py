# webhook_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, request
import threading

app = Flask(__name__)

class FormWebhookNode(Node):
    def __init__(self):
        super().__init__('form_webhook_node')

        self.pub = self.create_publisher(
            String,
            '/hospital/patiend_data',
            10
        )

        @app.route('/form', methods=['POST'])
        def form_webhook():
            data = request.get_json(force=True)

            self.get_logger().info(f'RAW JSON: {data}')

            if not data or 'patient_id' not in data:
                self.get_logger().error('❌ patient_id 없음')
                return 'Bad Request', 400

            patient_id = data['patient_id']   # ✅ 진짜 값만

            msg = String()
            msg.data = f'patient_id={patient_id}'
            self.pub.publish(msg)

            self.get_logger().info(
                f'📨 Google Form → system_start 발행: patient_id={patient_id}'
            )

            return 'OK', 200


def main():
    rclpy.init()
    node = FormWebhookNode()

    threading.Thread(
        target=lambda: app.run(
            host='0.0.0.0',
            port=5000,
            debug=False,
            use_reloader=False
        ),
        daemon=True
    ).start()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

