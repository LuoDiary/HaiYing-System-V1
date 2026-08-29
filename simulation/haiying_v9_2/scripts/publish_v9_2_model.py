#!/usr/bin/env python3
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


MODEL_TOPIC = "/haiying_v9_2/model_description"
PLACEHOLDER = "__SO101_CONTROLLERS_YAML__"


class ModelPublisher(Node):
    def __init__(self):
        super().__init__("haiying_v9_2_model_publisher")

        package_share = Path(
            get_package_share_directory("haiying_v9_2")
        )
        so101_share = Path(
            get_package_share_directory("so-101_description")
        )

        model_path = (
            package_share
            / "models"
            / "custom_quad_333_v9_2"
            / "model.sdf"
        )
        controller_path = (
            so101_share
            / "config"
            / "ros2_controllers.yaml"
        )

        if not model_path.is_file():
            raise FileNotFoundError(f"model not found: {model_path}")

        if not controller_path.is_file():
            raise FileNotFoundError(
                f"controller configuration not found: {controller_path}"
            )

        model_xml = model_path.read_text(encoding="utf-8")

        if model_xml.count(PLACEHOLDER) != 1:
            raise RuntimeError(
                "model must contain exactly one controller placeholder"
            )

        self._model_xml = model_xml.replace(
            PLACEHOLDER,
            str(controller_path),
            1,
        )

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._publisher = self.create_publisher(
            String,
            MODEL_TOPIC,
            qos,
        )
        self._first_publish = True
        self._timer = self.create_timer(0.5, self._publish)

        self.get_logger().info(
            f"V9_2_MODEL_READY topic={MODEL_TOPIC}"
        )

    def _publish(self):
        message = String(data=self._model_xml)
        self._publisher.publish(message)

        if self._first_publish:
            self.get_logger().info("V9_2_MODEL_PUBLISHED")
            self._first_publish = False


def main(args=None):
    rclpy.init(args=args)
    node = ModelPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
