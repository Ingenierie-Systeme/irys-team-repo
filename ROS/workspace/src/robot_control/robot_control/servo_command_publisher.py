import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ServoCommandPublisher(Node):
    """Publie une consigne sinusoïdale de servo en degrés."""

    AMPLITUDE_DEGREES = 40.0
    PERIOD_SECONDS = 5.0
    PUBLISH_RATE_HZ = 50.0

    def __init__(self) -> None:
        super().__init__('servo_command_publisher')
        self.publisher = self.create_publisher(Float64, '/servo_angle_cmd', 10)
        self.start_time = time.monotonic()
        self.timer = self.create_timer(1.0 / self.PUBLISH_RATE_HZ, self.publish_command)
        self.get_logger().info(
            'Publication d’une consigne sinusoïdale sur /servo_angle_cmd : '
            '±40 degrés, période de 5 secondes.'
        )

    def publish_command(self) -> None:
        elapsed = time.monotonic() - self.start_time
        message = Float64()
        message.data = self.AMPLITUDE_DEGREES * math.sin(
            2.0 * math.pi * elapsed / self.PERIOD_SECONDS
        )
        self.publisher.publish(message)


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
