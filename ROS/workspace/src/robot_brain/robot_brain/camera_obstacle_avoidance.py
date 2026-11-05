"""Choisit une direction locale à partir du flux optique épars (Lucas-Kanade)."""

import time
import numpy as np
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

# Assure-toi que ces fonctions correspondent au script que nous venons de modifier
from robot_brain.optical_flow_ttc import (
    choose_safe_direction,
    estimate_sector_time_to_contact_sparse,
)

from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class CameraObstacleAvoidance(Node):
    """Suit le cap avant et dévie vers le secteur visuellement le plus sûr via Lucas-Kanade."""

    def __init__(self) -> None:
        super().__init__('camera_obstacle_avoidance')
        self.declare_parameter('image_topic', '/input/camera')
        self.declare_parameter('direction_topic', '/robot/obstacle_direction')
        self.declare_parameter('emergency_stop_topic', '/safety/emergency_stop')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('minimum_time_to_contact', 2.0)
        self.declare_parameter('image_timeout_seconds', 0.5)

        self.image_topic = self.get_parameter('image_topic').value
        self.direction_topic = self.get_parameter('direction_topic').value
        self.emergency_stop_topic = self.get_parameter('emergency_stop_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.minimum_time_to_contact = float(self.get_parameter('minimum_time_to_contact').value)
        self.image_timeout_seconds = float(self.get_parameter('image_timeout_seconds').value)

        if self.minimum_time_to_contact <= 0.0:
            raise ValueError('minimum_time_to_contact doit être supérieur à zéro.')
        if self.image_timeout_seconds <= 0.0:
            raise ValueError('image_timeout_seconds doit être supérieur à zéro.')

        self.bridge = CvBridge()
        self.previous_gray = None
        self.previous_image_time: float | None = None
        self.last_image_time: float | None = None
        self.emergency_stop = False
        self.last_direction: tuple[float, float, float] | None = None

        # --- Paramètres Lucas-Kanade ---
        self.feature_params = dict(
            maxCorners=100,
            qualityLevel=0.3,
            minDistance=7,
            blockSize=7
        )
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )
        self.p0 = None # Points à suivre
        # -------------------------------

        direction_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.direction_publisher = self.create_publisher(
            Vector3Stamped, self.direction_topic, direction_qos
        )
        self.image_subscription = self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data,
        )
        emergency_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.stop_subscription = self.create_subscription(
            Bool, self.emergency_stop_topic, self.on_emergency_stop, emergency_qos,
        )
        self.timeout_timer = self.create_timer(
            min(0.1, self.image_timeout_seconds / 2.0), self.check_image_timeout,
        )
        self.publish_direction((0.0, 0.0, 0.0))
        self.get_logger().info('Évitement caméra LK initialisé.')

    def on_image(self, message: Image) -> None:
        now = time.monotonic()
        try:
            gray = self.bridge.imgmsg_to_cv2(message, desired_encoding='mono8')
        except CvBridgeError as error:
            self.get_logger().error(f'Image caméra invalide : {error}')
            self.reset_vision(now)
            return

        # Gestion du timeout d'image
        if self.last_image_time is not None and now - self.last_image_time > self.image_timeout_seconds:
            self.previous_gray = None
            self.previous_image_time = None
            self.p0 = None

        self.last_image_time = now

        # Initialisation de la première frame ou changement de résolution
        if self.previous_gray is None or self.previous_image_time is None or gray.shape != self.previous_gray.shape:
            if self.previous_gray is not None and gray.shape != self.previous_gray.shape:
                self.get_logger().warning('Résolution modifiée ; réinitialisation.')
            
            self.previous_gray = gray
            self.previous_image_time = now
            self.p0 = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            self.publish_direction((0.0, 0.0, 0.0))
            return

        elapsed = now - self.previous_image_time
        if elapsed <= 0.0:
            return

        # Remplacement des points s'il y en a trop peu
        if self.p0 is None or len(self.p0) < 10:
            self.p0 = cv2.goodFeaturesToTrack(self.previous_gray, mask=None, **self.feature_params)
            
        if self.p0 is None or len(self.p0) == 0:
            self.reset_vision(now, gray)
            return

        # Calcul du flux optique Lucas-Kanade
        p1, st, err = cv2.calcOpticalFlowPyrLK(
            self.previous_gray, gray, self.p0, None, **self.lk_params
        )

        if p1 is not None and st is not None:
            good_new = p1[st == 1]
            good_old = self.p0[st == 1]
        else:
            good_new, good_old = np.array([]), np.array([])

        if len(good_new) == 0:
            self.reset_vision(now, gray)
            return

        # Préparation des données pour le calcul du TTC
        velocities = (good_new - good_old) / elapsed
        height, width = gray.shape

        try:
            sectors = estimate_sector_time_to_contact_sparse(
                points=good_new,
                velocities=velocities,
                image_width=width,
                image_height=height
            )
        except Exception as error:
            self.get_logger().error(f'Erreur estimation TTC : {error}')
            self.reset_vision(now, gray, good_new)
            return

        direction = choose_safe_direction(sectors, self.minimum_time_to_contact)
        self.publish_direction(direction)

        # Mise à jour pour la prochaine frame
        self.previous_gray = gray
        self.previous_image_time = now
        self.p0 = good_new.reshape(-1, 1, 2)

    def reset_vision(self, now: float, gray_img=None, points=None) -> None:
        """Méthode utilitaire pour réinitialiser l'état en cas d'erreur/manque de données."""
        self.previous_gray = gray_img
        self.previous_image_time = now if gray_img is not None else None
        if points is not None and len(points) > 0:
            self.p0 = points.reshape(-1, 1, 2)
        else:
            self.p0 = None
        self.publish_direction((0.0, 0.0, 0.0))

    def on_emergency_stop(self, message: Bool) -> None:
        self.emergency_stop = message.data
        if self.emergency_stop:
            self.publish_direction((0.0, 0.0, 0.0))

    def check_image_timeout(self) -> None:
        if (
            self.last_image_time is None
            or time.monotonic() - self.last_image_time > self.image_timeout_seconds
        ):
            self.previous_gray = None
            self.previous_image_time = None
            self.p0 = None
            self.publish_direction((0.0, 0.0, 0.0))

    def publish_direction(self, direction: tuple[float, float, float]) -> None:
        if self.emergency_stop:
            direction = (0.0, 0.0, 0.0)

        message = Vector3Stamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.vector.x, message.vector.y, message.vector.z = direction
        self.direction_publisher.publish(message)
        
        changed = direction != self.last_direction
        self.last_direction = direction
        if changed and direction != (0.0, 0.0, 0.0):
            self.get_logger().info(f'Direction locale : x={direction[0]:.2f}, y={direction[1]:.2f}.')


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
