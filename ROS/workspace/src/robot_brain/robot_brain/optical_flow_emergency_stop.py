"""Détecte une approche d'obstacle et publie l'état d'arrêt d'urgence.
Le temps avat impact est déterminé par calcul du flux optique
"""

import time

import cv2

from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from robot_brain.optical_flow_ttc import estimate_time_to_contact

from sensor_msgs.msg import Image

from std_msgs.msg import Bool


class OpticalFlowEmergencyStop(Node):
    """Publie un arrêt lorsque le TTC estimé devient inférieur au seuil."""

    def __init__(self) -> None:
        super().__init__('optical_flow_emergency_stop')
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('stop_topic', '/safety/emergency_stop')
        self.declare_parameter('ttc_threshold_seconds', 2.0)
        self.declare_parameter('image_timeout_seconds', 0.5)

        self.image_topic = self.get_parameter('image_topic').value
        self.stop_topic = self.get_parameter('stop_topic').value
        self.ttc_threshold_seconds = float(
            self.get_parameter('ttc_threshold_seconds').value
        )
        self.image_timeout_seconds = float(
            self.get_parameter('image_timeout_seconds').value
        )
        if self.ttc_threshold_seconds <= 0.0:
            raise ValueError(
                'ttc_threshold_seconds doit être supérieur à zéro.'
            )
        if self.image_timeout_seconds <= 0.0:
            raise ValueError(
                'image_timeout_seconds doit être supérieur à zéro.'
            )

        stop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.stop_publisher = self.create_publisher(
            Bool, self.stop_topic, stop_qos
        )
        self.bridge = CvBridge()
        self.previous_gray = None
        self.previous_image_time = None
        self.last_image_time = None
        self.stop_active = None

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            qos_profile_sensor_data,
        )
        self.timeout_timer = self.create_timer(0.1, self.check_image_timeout)
        self.heartbeat_timer = self.create_timer(
            0.1,
            self.publish_heartbeat,
        )

        self.publish_stop(True, 'en attente de deux images caméra valides')
        self.get_logger().info(
            f'Estimation TTC depuis {self.image_topic}; '
            f'arrêt publié sur {self.stop_topic} si TTC <= '
            f'{self.ttc_threshold_seconds:.2f} s.'
        )

    def on_image(self, message: Image) -> None:
        now = time.monotonic()
        try:
            gray = self.bridge.imgmsg_to_cv2(message, desired_encoding='mono8')
        except CvBridgeError as error:
            self.publish_stop(True, f'image caméra invalide : {error}')
            self.previous_gray = None
            self.last_image_time = None
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
            self.publish_stop(
                True,
                'attente de la prochaine image pour calculer le flux',
            )
            return

        elapsed = now - self.previous_image_time
        if elapsed <= 0.0:
            self.publish_stop(True, 'intervalle d’images invalide')
            self.previous_gray = gray
            self.previous_image_time = now
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
            self.publish_stop(
                True,
                f'échec du calcul de flux optique : {error}',
            )
            self.previous_gray = gray
            self.previous_image_time = now
            return

        self.previous_gray = gray
        self.previous_image_time = now
        time_to_contact = estimate_time_to_contact(flow / elapsed)
        if (
            time_to_contact is not None
            and time_to_contact <= self.ttc_threshold_seconds
        ):
            self.publish_stop(
                True,
                f'obstacle détecté, TTC estimé à {time_to_contact:.2f} s',
            )
        else:
            self.publish_stop(False, 'aucune approche dangereuse estimée')

    def check_image_timeout(self) -> None:
        if (
            self.last_image_time is None
            or time.monotonic() - self.last_image_time
            > self.image_timeout_seconds
        ):
            self.previous_gray = None
            self.previous_image_time = None
            self.publish_stop(True, 'flux caméra absent ou expiré')

    def publish_heartbeat(self) -> None:
        self.publish_stop(
            self.stop_active is not False,
            'heartbeat',
            force=True,
        )

    def publish_stop(
        self,
        active: bool,
        reason: str,
        force: bool = False,
    ) -> None:
        changed = self.stop_active != active
        if not changed and not force:
            return

        message = Bool()
        message.data = active
        self.stop_publisher.publish(message)
        self.stop_active = active
        if not changed:
            return
        if active:
            self.get_logger().warning(
                f'ARRÊT D’URGENCE actif : {reason}'
            )
        else:
            self.get_logger().info(f'Arrêt d’urgence désactivé : {reason}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OpticalFlowEmergencyStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
