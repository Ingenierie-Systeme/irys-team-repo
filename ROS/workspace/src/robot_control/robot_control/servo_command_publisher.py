import math
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray

from robot_control.servo_command import convert_direction_to_servo_positions

PUBLISH_RATE_HZ = 50.0


class ServoCommandPublisher(Node):
    """Convertit la direction du robot en consignes pour les deux servos
    Code de de test pour le contrôle du servo."""

    def __init__(self) -> None:
        super().__init__('servo_command_publisher')
        self.declare_parameter('direction_topic', '/robot/direction')
        self.declare_parameter('direction_timeout_seconds', 1.0)
        direction_topic = self.get_parameter('direction_topic').value
        self.direction_timeout_seconds = float(
            self.get_parameter('direction_timeout_seconds').value
        )
        if (
            not math.isfinite(self.direction_timeout_seconds)
            or self.direction_timeout_seconds <= 0.0
        ):
            raise ValueError(
                'direction_timeout_seconds doit être supérieur à zéro.'
            )

        direction_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.servo_publisher = self.create_publisher(
            Float32MultiArray, '/output/servos', 10
        )
        self.direction_subscription = self.create_subscription(
            Vector3Stamped,
            direction_topic,
            self.on_direction,
            direction_qos,
        )
        self.direction = (0.0, 0.0)
        self.last_direction_time: float | None = None
        self.start_time = time.monotonic()
        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ,
            self.publish_commands,
        )
        self.get_logger().info(
            f'Conversion de {direction_topic} vers /output/servos '
            '(nageoire, masse mobile).'
        )

    def on_direction(self, message: Vector3Stamped) -> None:
        direction = (message.vector.x, message.vector.y)
        if not all(math.isfinite(value) for value in direction):
            self.direction = (0.0, 0.0)
            self.last_direction_time = time.monotonic()
            self.get_logger().error(
                'Direction invalide reçue ; consigne servo mise à zéro.'
            )
            return

        self.direction = direction
        self.last_direction_time = time.monotonic()

    def publish_commands(self) -> None:
        now = time.monotonic()
        elapsed = now - self.start_time
        direction = self.direction
        if (
            self.last_direction_time is None
            or now - self.last_direction_time
            > self.direction_timeout_seconds
        ):
            direction = (0.0, 0.0)
        fin_angle, mass_angle = convert_direction_to_servo_positions(
            *direction,
            elapsed,
        )

        message = Float32MultiArray()
        message.data = [float(fin_angle), float(mass_angle)]
        self.servo_publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ServoCommandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
