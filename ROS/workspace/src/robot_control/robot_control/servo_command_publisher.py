import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float64

from robot_control.servo_command import convert_direction_to_servo_positions

PUBLISH_RATE_HZ = 50.0


class ServoCommandPublisher(Node):
    """Convertit la direction du robot en consignes pour les deux servos
    Code de de test pour le contrôle du servo."""

    def __init__(self) -> None:
        super().__init__('servo_command_publisher')
        self.declare_parameter('direction_topic', '/robot/direction')
        direction_topic = self.get_parameter('direction_topic').value

        direction_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.fin_publisher = self.create_publisher(
            Float64, '/servo/nageoire_cmd', 10
        )
        self.mass_publisher = self.create_publisher(
            Float64, '/servo/masse_cmd', 10
        )
        self.direction_subscription = self.create_subscription(
            Vector3Stamped,
            direction_topic,
            self.on_direction,
            direction_qos,
        )
        self.direction = (0.0, 0.0)
        self.start_time = time.monotonic()
        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ,
            self.publish_commands,
        )
        self.get_logger().info(
            f'Conversion de {direction_topic} vers /servo/nageoire_cmd et '
            '/servo/masse_cmd.'
        )

    def on_direction(self, message: Vector3Stamped) -> None:
        self.direction = (message.vector.x, message.vector.y)

    def publish_commands(self) -> None:
        elapsed = time.monotonic() - self.start_time
        fin_angle, mass_angle = convert_direction_to_servo_positions(
            *self.direction,
            elapsed,
        )

        fin_message = Float64()
        fin_message.data = fin_angle
        self.fin_publisher.publish(fin_message)

        mass_message = Float64()
        mass_message.data = mass_angle
        self.mass_publisher.publish(mass_message)


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
