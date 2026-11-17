"""Arbitre l'état estimé, l'évitement caméra et l'arrêt d'urgence."""

import math
import time

from geometry_msgs.msg import Vector3Stamped

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from robot_brain.brain_logic import (
    Direction,
    STOP_DIRECTION,
    apply_safety_gate,
)
from std_msgs.msg import Bool


class RobotBrain(Node):
    """Transmet la direction caméra seulement avec des entrées sûres."""

    def __init__(self) -> None:
        super().__init__('robot_brain')
        self.declare_parameter('pose_topic', '/robot/pose')
        self.declare_parameter(
            'obstacle_direction_topic',
            '/robot/obstacle_direction',
        )
        self.declare_parameter('direction_topic', '/commande/direction')
        self.declare_parameter('body_frame', 'base_link')
        self.declare_parameter('input_timeout_seconds', 0.5)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.obstacle_direction_topic = self.get_parameter(
            'obstacle_direction_topic'
        ).value

        self.direction_topic = self.get_parameter('direction_topic').value
        self.body_frame = self.get_parameter('body_frame').value
        self.input_timeout_seconds = float(
            self.get_parameter('input_timeout_seconds').value
        )
        if (
            not math.isfinite(self.input_timeout_seconds)
            or self.input_timeout_seconds <= 0.0
        ):
            raise ValueError(
                'input_timeout_seconds doit être supérieur à zéro.'
            )

        self.last_pose_stamp_ns: int | None = None
        self.last_pose_time: float | None = None
        self.last_direction: Direction | None = None
        self.last_direction_time: float | None = None
        self.emergency_stop: bool | None = None
        self.last_emergency_stop_time: float | None = None
        self.last_block_reason: str | None = None
        self.warned_invalid_pose = False
        self.warned_invalid_direction = False

        direction_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.direction_publisher = self.create_publisher(
            Vector3Stamped,
            self.direction_topic,
            direction_qos,
        )
        self.pose_subscription = self.create_subscription(
            Odometry,
            self.pose_topic,
            self.on_pose,
            10,
        )
        self.obstacle_direction_subscription = self.create_subscription(
            Vector3Stamped,
            self.obstacle_direction_topic,
            self.on_obstacle_direction,
            direction_qos,
        )

        self.timer = self.create_timer(0.1, self.publish_control_direction)
        self.publish_control_direction()
        self.get_logger().info(
            f'Arbitrage {self.pose_topic}, {self.obstacle_direction_topic} '
        )

    def on_pose(self, message: Odometry) -> None:
        if not self.pose_is_valid(message):
            self.last_pose_time = None
            if not self.warned_invalid_pose:
                self.get_logger().warning(
                    'Pose estimée invalide ou dans un repère inattendu.'
                )
                self.warned_invalid_pose = True
            return

        stamp_ns = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        if stamp_ns != self.last_pose_stamp_ns:
            self.last_pose_stamp_ns = stamp_ns
            self.last_pose_time = time.monotonic()
        self.warned_invalid_pose = False

    def pose_is_valid(self, message: Odometry) -> bool:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        orientation_norm = math.sqrt(
            orientation.x**2
            + orientation.y**2
            + orientation.z**2
            + orientation.w**2
        )
        return (
            message.child_frame_id == self.body_frame
            and all(math.isfinite(value) for value in values)
            and math.isfinite(orientation_norm)
            and orientation_norm > 1e-6
            and (
                message.header.stamp.sec != 0
                or message.header.stamp.nanosec != 0
            )
        )

    def on_obstacle_direction(self, message: Vector3Stamped) -> None:
        values = (
            message.vector.x,
            message.vector.y,
            message.vector.z,
        )
        if (
            message.header.frame_id != self.body_frame
            or not all(math.isfinite(value) for value in values)
        ):
            self.last_direction = None
            self.last_direction_time = None
            if not self.warned_invalid_direction:
                self.get_logger().warning(
                    'Direction caméra invalide ou dans un repère inattendu.'
                )
                self.warned_invalid_direction = True
            return

        self.last_direction = values
        self.last_direction_time = time.monotonic()
        self.warned_invalid_direction = False


    def publish_control_direction(self) -> None:
        now = time.monotonic()

        pose_is_fresh = (
            self.last_pose_time is not None
            and now - self.last_pose_time <= self.input_timeout_seconds
        )
        direction_is_fresh = (
            self.last_direction_time is not None
            and now - self.last_direction_time <= self.input_timeout_seconds
        )

        if not pose_is_fresh:
            direction = STOP_DIRECTION
            reason = 'pose estimée absente, invalide ou périmée'
        elif not direction_is_fresh or self.last_direction is None:
            direction = STOP_DIRECTION
            reason = 'direction caméra absente ou périmée'
        else:
            direction = apply_safety_gate(
                self.last_direction,
                state_ready=True,
                emergency_stop=False,
            )
            reason = ''

        output = Vector3Stamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.body_frame
        (
            output.vector.x,
            output.vector.y,
            output.vector.z,
        ) = direction
        self.direction_publisher.publish(output)

        if reason and reason != self.last_block_reason:
            self.get_logger().warning(
                f'Consigne de déplacement nulle : {reason}.'
            )
        self.last_block_reason = reason or None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()