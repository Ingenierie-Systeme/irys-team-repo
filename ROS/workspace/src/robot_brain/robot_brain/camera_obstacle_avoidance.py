"""Choisit une direction locale à partir du flux optique caméra."""

import time

import cv2

from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Vector3Stamped

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from robot_brain.optical_flow_ttc import (
    choose_safe_direction,
    estimate_sector_time_to_contact,
)

from sensor_msgs.msg import Image

from std_msgs.msg import Bool


class CameraObstacleAvoidance(Node):
    """Suit le cap avant et dévie vers le secteur visuellement le plus sûr."""

    def __init__(self) -> None:
        super().__init__('camera_obstacle_avoidance')
        self.declare_parameter('image_topic', '/camera/image')  # réception de la caméra

        self.declare_parameter(
            'direction_topic',
            '/robot/obstacle_direction',
        )
        self.declare_parameter(
            'emergency_stop_topic', '/safety/emergency_stop'
        ) # arrêt d'urgence
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('minimum_time_to_contact', 2.0)
        self.declare_parameter('image_timeout_seconds', 0.5)

        self.image_topic = self.get_parameter('image_topic').value
        self.direction_topic = self.get_parameter('direction_topic').value
        self.emergency_stop_topic = self.get_parameter(
            'emergency_stop_topic'
        ).value
        self.base_frame = self.get_parameter('base_frame').value
        self.minimum_time_to_contact = float(
            self.get_parameter('minimum_time_to_contact').value
        )
        self.image_timeout_seconds = float(
            self.get_parameter('image_timeout_seconds').value
        )
        if self.minimum_time_to_contact <= 0.0:
            raise ValueError(
                'minimum_time_to_contact doit être supérieur à zéro.'
            )
        if self.image_timeout_seconds <= 0.0:
            raise ValueError(
                'image_timeout_seconds doit être supérieur à zéro.'
            )

        self.bridge = CvBridge()
        self.previous_gray = None
        self.previous_image_time: float | None = None
        self.last_image_time: float | None = None
        self.emergency_stop = False
        self.last_direction: tuple[float, float, float] | None = None

        direction_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.direction_publisher = self.create_publisher(
            Vector3Stamped, self.direction_topic, direction_qos
        )
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            qos_profile_sensor_data,
        )
        emergency_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.stop_subscription = self.create_subscription(
            Bool,
            self.emergency_stop_topic,
            self.on_emergency_stop,
            emergency_qos,
        )
        self.timeout_timer = self.create_timer(
            min(0.1, self.image_timeout_seconds / 2.0),
            self.check_image_timeout,
        )
        self.publish_direction((0.0, 0.0, 0.0))
        self.get_logger().info(
            f'Évitement caméra: {self.image_topic} -> '
            f'{self.direction_topic} (TTC minimal '
            f'{self.minimum_time_to_contact:.2f} s).'
        )

    def on_image(self, message: Image) -> None:
        now = time.monotonic()
        try:
            gray = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='mono8',
            )
        except CvBridgeError as error:
            self.get_logger().error(f'Image caméra invalide : {error}')
            self.previous_gray = None
            self.previous_image_time = None
            self.last_image_time = None
            self.publish_direction((0.0, 0.0, 0.0))
            return

        if (
            self.last_image_time is not None
            and now - self.last_image_time > self.image_timeout_seconds
        ):
            self.previous_gray = None
            self.previous_image_time = None

        self.last_image_time = now
        if self.previous_gray is None or self.previous_image_time is None:
            self.previous_gray = gray
            self.previous_image_time = now
            self.publish_direction((0.0, 0.0, 0.0))
            return

        if gray.shape != self.previous_gray.shape:
            self.get_logger().warning(
                'Résolution caméra modifiée ; réinitialisation '
                'du flux optique.'
            )
            self.previous_gray = gray
            self.previous_image_time = now
            self.publish_direction((0.0, 0.0, 0.0))
            return

        elapsed = now - self.previous_image_time
        if elapsed <= 0.0:
            self.previous_gray = gray
            self.previous_image_time = now
            self.publish_direction((0.0, 0.0, 0.0))
            return

        try:
            flow = cv2.calcOpticalFlowFarneback(
                self.previous_gray,
                gray,
                None,
                0.5,
                3,
                15,
                3,
                5,
                1.2,
                0,
            )
        except cv2.error as error:
            self.get_logger().error(f'Échec du flux optique : {error}')
            self.previous_gray = gray
            self.previous_image_time = now
            self.publish_direction((0.0, 0.0, 0.0))
            return

        self.previous_gray = gray
        self.previous_image_time = now
        try:
            sectors = estimate_sector_time_to_contact(flow / elapsed)
        except ValueError as error:
            self.get_logger().error(f'Image inutilisable : {error}')
            self.publish_direction((0.0, 0.0, 0.0))
            return
        direction = choose_safe_direction(
            sectors,
            self.minimum_time_to_contact,
        )
        self.publish_direction(direction)

    def on_emergency_stop(self, message: Bool) -> None:
        self.emergency_stop = message.data
        if self.emergency_stop:
            self.publish_direction((0.0, 0.0, 0.0))

    def check_image_timeout(self) -> None:
        if (
            self.last_image_time is None
            or time.monotonic() - self.last_image_time
            > self.image_timeout_seconds
        ):
            self.previous_gray = None
            self.previous_image_time = None
            self.publish_direction((0.0, 0.0, 0.0))

    def publish_direction(
        self,
        direction: tuple[float, float, float],
    ) -> None:
        if self.emergency_stop:
            direction = (0.0, 0.0, 0.0)

        message = Vector3Stamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.vector.x, message.vector.y, message.vector.z = direction
        self.direction_publisher.publish(message)
        changed = direction != self.last_direction
        self.last_direction = direction
        if not changed:
            return
        if direction == (0.0, 0.0, 0.0):
            self.get_logger().warning(
                'Aucune direction sûre ou perception caméra indisponible.'
            )
        else:
            self.get_logger().info(
                f'Direction locale sélectionnée : '
                f'x={direction[0]:.2f}, y={direction[1]:.2f}.'
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
