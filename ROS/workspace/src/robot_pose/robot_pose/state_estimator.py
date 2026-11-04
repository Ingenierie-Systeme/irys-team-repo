"""Fusion IMU-GNSS et publication de l'odométrie estimée du robot."""

import math
import time

from builtin_interfaces.msg import Time as TimeMessage

from nav_msgs.msg import Odometry

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from robot_pose.geometry import (
    GRAVITY_METERS_PER_SECOND_SQUARED,
    geodetic_to_enu,
    pressure_to_depth,
    quaternion_rotate,
)
from robot_pose.kalman_filter import PositionVelocityFilter

from sensor_msgs.msg import FluidPressure, Imu, NavSatFix, NavSatStatus


def message_timestamp(stamp: TimeMessage) -> float:
    """Convertit un timestamp ROS en secondes, ou utilise l'horloge système."""
    if stamp.sec == 0 and stamp.nanosec == 0:
        return time.time()
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class RobotStateEstimator(Node):
    """Estimate ENU position and velocity from IMU, GNSS, and pressure."""

    def __init__(self) -> None:
        super().__init__('robot_state_estimator')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('gnss_topic', '/gnss/fix')
        self.declare_parameter('pressure_topic', '/pressure')
        self.declare_parameter('pose_topic', '/robot/pose')
        self.declare_parameter('origin_latitude', 48.39)
        self.declare_parameter('origin_longitude', -4.48)
        self.declare_parameter('origin_altitude', 0.0)
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('body_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('acceleration_noise_stddev', 0.5)
        self.declare_parameter('initial_velocity_variance', 25.0)
        self.declare_parameter('default_gnss_position_variance', 4.0)
        self.declare_parameter('gnss_mahalanobis_threshold', 16.27)
        self.declare_parameter('water_density_kg_m3', 1000.0)
        self.declare_parameter('default_pressure_depth_variance', 1.0)
        self.declare_parameter('pressure_mahalanobis_threshold', 16.27)
        self.declare_parameter('gnss_reacquisition_timeout', 2.0)
        self.declare_parameter('max_gnss_reacquisition_distance', 50.0)
        self.declare_parameter('imu_includes_gravity', True)

        self.imu_topic = self.get_parameter('imu_topic').value
        self.gnss_topic = self.get_parameter('gnss_topic').value
        self.pressure_topic = self.get_parameter('pressure_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.origin_latitude = float(
            self.get_parameter('origin_latitude').value
        )
        self.origin_longitude = float(
            self.get_parameter('origin_longitude').value
        )
        self.origin_altitude = float(
            self.get_parameter('origin_altitude').value
        )
        self.world_frame = self.get_parameter('world_frame').value
        self.body_frame = self.get_parameter('body_frame').value
        self.publish_rate_hz = float(
            self.get_parameter('publish_rate_hz').value
        )
        self.acceleration_noise_stddev = float(
            self.get_parameter('acceleration_noise_stddev').value
        )
        self.default_gnss_position_variance = float(
            self.get_parameter('default_gnss_position_variance').value
        )
        self.gnss_mahalanobis_threshold = float(
            self.get_parameter('gnss_mahalanobis_threshold').value
        )
        self.water_density_kg_m3 = float(
            self.get_parameter('water_density_kg_m3').value
        )
        self.default_pressure_depth_variance = float(
            self.get_parameter('default_pressure_depth_variance').value
        )
        self.pressure_mahalanobis_threshold = float(
            self.get_parameter('pressure_mahalanobis_threshold').value
        )
        self.gnss_reacquisition_timeout = float(
            self.get_parameter('gnss_reacquisition_timeout').value
        )
        self.max_gnss_reacquisition_distance = float(
            self.get_parameter('max_gnss_reacquisition_distance').value
        )
        self.imu_includes_gravity = bool(
            self.get_parameter('imu_includes_gravity').value
        )
        initial_velocity_variance = float(
            self.get_parameter('initial_velocity_variance').value
        )
        if not -90.0 < self.origin_latitude < 90.0:
            raise ValueError(
                'origin_latitude doit être entre -90 et 90 degrés.'
            )
        if not -180.0 <= self.origin_longitude <= 180.0:
            raise ValueError(
                'origin_longitude doit être entre -180 et 180 degrés.'
            )
        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz doit être supérieur à zéro.')
        if self.acceleration_noise_stddev <= 0.0:
            raise ValueError(
                'acceleration_noise_stddev doit être supérieur à zéro.'
            )
        if initial_velocity_variance <= 0.0:
            raise ValueError(
                'initial_velocity_variance doit être supérieur à zéro.'
            )
        if self.default_gnss_position_variance <= 0.0:
            raise ValueError(
                'default_gnss_position_variance doit être supérieur à zéro.'
            )
        if self.gnss_mahalanobis_threshold <= 0.0:
            raise ValueError(
                'gnss_mahalanobis_threshold doit être supérieur à zéro.'
            )
        if (
            not math.isfinite(self.water_density_kg_m3)
            or self.water_density_kg_m3 <= 0.0
        ):
            raise ValueError(
                'water_density_kg_m3 doit être supérieur à zéro.'
            )
        if (
            not math.isfinite(self.default_pressure_depth_variance)
            or self.default_pressure_depth_variance <= 0.0
        ):
            raise ValueError(
                'default_pressure_depth_variance doit être supérieur à zéro.'
            )
        if (
            not math.isfinite(self.pressure_mahalanobis_threshold)
            or self.pressure_mahalanobis_threshold <= 0.0
        ):
            raise ValueError(
                'pressure_mahalanobis_threshold doit être supérieur à zéro.'
            )
        if self.gnss_reacquisition_timeout <= 0.0:
            raise ValueError(
                'gnss_reacquisition_timeout doit être supérieur à zéro.'
            )
        if self.max_gnss_reacquisition_distance <= 0.0:
            raise ValueError(
                'max_gnss_reacquisition_distance doit être supérieur à zéro.'
            )

        self.filter = PositionVelocityFilter(initial_velocity_variance)
        self.orientation: np.ndarray | None = None
        self.orientation_covariance = np.zeros(9, dtype=np.float64)
        self.angular_velocity = np.zeros(3, dtype=np.float64)
        self.angular_velocity_covariance = np.zeros(9, dtype=np.float64)
        self.last_acceleration = np.zeros(3, dtype=np.float64)
        self.last_imu_time: float | None = None
        self.last_imu_stamp: TimeMessage | None = None
        self.last_gnss_time: float | None = None
        self.last_pressure_time: float | None = None
        self.surface_pressure_pa: float | None = None
        self.warned_invalid_imu = False
        self.warned_stale_imu = False
        self.warned_gnss_outlier = False
        self.warned_invalid_pressure = False
        self.warned_invalid_pressure_variance = False
        self.warned_pressure_outlier = False

        self.pose_publisher = self.create_publisher(
            Odometry, self.pose_topic, 10
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self.on_imu,
            qos_profile_sensor_data,
        )
        self.gnss_subscription = self.create_subscription(
            NavSatFix,
            self.gnss_topic,
            self.on_gnss,
            qos_profile_sensor_data,
        )
        self.pressure_subscription = self.create_subscription(
            FluidPressure,
            self.pressure_topic,
            self.on_pressure,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.publish_pose,
        )
        self.get_logger().info(
            f'Estimation de pose: {self.imu_topic} + {self.gnss_topic} + '
            f'{self.pressure_topic} -> {self.pose_topic} '
            f'({self.world_frame} -> {self.body_frame}).'
        )

    def on_imu(self, message: Imu) -> None:
        stamp = message_timestamp(message.header.stamp)
        if self.last_imu_time is not None and stamp <= self.last_imu_time:
            return

        quaternion = np.array([
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        ], dtype=np.float64)
        acceleration = np.array([
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        ], dtype=np.float64)
        angular_velocity = np.array([
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        ], dtype=np.float64)
        values = np.concatenate((quaternion, acceleration, angular_velocity))
        norm = float(np.linalg.norm(quaternion))
        if not np.all(np.isfinite(values)) or norm < 1e-6:
            if not self.warned_invalid_imu:
                self.get_logger().warning(
                    'Mesure IMU ignorée : quaternion ou valeurs invalides.'
                )
                self.warned_invalid_imu = True
            return

        quaternion /= norm
        acceleration_world = quaternion_rotate(quaternion, acceleration)
        if self.imu_includes_gravity:
            acceleration_world[2] -= GRAVITY_METERS_PER_SECOND_SQUARED

        if self.filter.initialized and self.last_imu_time is not None:
            delta_time = stamp - self.last_imu_time
            if delta_time > 0.5:
                if not self.warned_stale_imu:
                    self.get_logger().warning(
                        'Intervalle IMU supérieur à 0,5 s : propagation '
                        'sans réutiliser une accélération périmée.'
                    )
                    self.warned_stale_imu = True
                self.filter.predict(
                    np.zeros(3, dtype=np.float64),
                    delta_time,
                    self.acceleration_noise_stddev,
                )
            elif delta_time > 0.0:
                self.filter.predict(
                    self.last_acceleration,
                    delta_time,
                    self.acceleration_noise_stddev,
                )

        self.orientation = quaternion
        self.orientation_covariance = np.asarray(
            message.orientation_covariance, dtype=np.float64
        )
        self.angular_velocity = angular_velocity
        self.angular_velocity_covariance = np.asarray(
            message.angular_velocity_covariance, dtype=np.float64
        )
        self.last_acceleration = acceleration_world
        self.last_imu_time = stamp
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            seconds = int(stamp)
            self.last_imu_stamp = TimeMessage(
                sec=seconds,
                nanosec=int((stamp - seconds) * 1e9),
            )
        else:
            self.last_imu_stamp = TimeMessage(
                sec=message.header.stamp.sec,
                nanosec=message.header.stamp.nanosec,
            )

    def on_pressure(self, message: FluidPressure) -> None:
        pressure_pa = float(message.fluid_pressure)
        if not math.isfinite(pressure_pa) or pressure_pa <= 0.0:
            if not self.warned_invalid_pressure:
                self.get_logger().warning(
                    'Mesure de pression ignorée : la pression doit être '
                    'finie et strictement positive.'
                )
                self.warned_invalid_pressure = True
            return

        stamp = message_timestamp(message.header.stamp)
        if (
            self.last_pressure_time is not None
            and stamp <= self.last_pressure_time
        ):
            return
        self.last_pressure_time = stamp

        if self.surface_pressure_pa is None:
            self.surface_pressure_pa = pressure_pa
            self.get_logger().info(
                f'Référence de pression de surface calibrée à '
                f'{pressure_pa:.1f} Pa.'
            )
            return

        depth = pressure_to_depth(
            pressure_pa,
            self.surface_pressure_pa,
            self.water_density_kg_m3,
        )
        pressure_variance = float(message.variance)
        if math.isfinite(pressure_variance) and pressure_variance > 0.0:
            depth_variance = pressure_variance / (
                self.water_density_kg_m3
                * GRAVITY_METERS_PER_SECOND_SQUARED
            )**2
        else:
            if (
                not math.isfinite(pressure_variance)
                or pressure_variance < 0.0
            ) and not self.warned_invalid_pressure_variance:
                self.get_logger().warning(
                    'Variance de pression invalide ; utilisation de '
                    'default_pressure_depth_variance.'
                )
                self.warned_invalid_pressure_variance = True
            depth_variance = self.default_pressure_depth_variance

        accepted = self.filter.update_vertical_position(
            -depth,
            depth_variance,
            self.pressure_mahalanobis_threshold,
        )
        if not accepted and self.filter.initialized:
            if not self.warned_pressure_outlier:
                self.get_logger().warning(
                    'Correction verticale de pression rejetée comme '
                    'valeur aberrante.'
                )
                self.warned_pressure_outlier = True
        elif accepted:
            self.warned_pressure_outlier = False

    def on_gnss(self, message: NavSatFix) -> None:
        if (
            message.status.status == NavSatStatus.STATUS_NO_FIX
            or not math.isfinite(message.latitude)
            or not math.isfinite(message.longitude)
            or not math.isfinite(message.altitude)
            or not -90.0 <= message.latitude <= 90.0
            or not -180.0 <= message.longitude <= 180.0
        ):
            return

        position = geodetic_to_enu(
            message.latitude,
            message.longitude,
            message.altitude,
            self.origin_latitude,
            self.origin_longitude,
            self.origin_altitude,
        )
        covariance = self.gnss_covariance(message)
        was_initialized = self.filter.initialized
        stamp = message_timestamp(message.header.stamp)
        if self.last_gnss_time is not None and stamp <= self.last_gnss_time:
            return
        reacquiring = (
            self.last_gnss_time is not None
            and stamp - self.last_gnss_time
            > self.gnss_reacquisition_timeout
        )
        if (
            reacquiring
            and np.linalg.norm(position - self.filter.state[:3])
            <= self.max_gnss_reacquisition_distance
        ):
            self.filter.reset_position(position, covariance)
            accepted = True
            self.get_logger().info(
                'Position recalibrée après la réacquisition du GNSS.'
            )
        else:
            accepted = self.filter.update_position(
                position,
                covariance,
                self.gnss_mahalanobis_threshold,
            )
        if not accepted:
            if not self.warned_gnss_outlier:
                self.get_logger().warning(
                    'Correction GNSS rejetée car trop éloignée '
                    'de la pose estimée.'
                )
                self.warned_gnss_outlier = True
        elif not was_initialized:
            self.get_logger().info(
                'Estimateur initialisé à partir du premier fix GNSS valide.'
            )
        if accepted:
            self.last_gnss_time = stamp
            self.warned_gnss_outlier = False

    def gnss_covariance(self, message: NavSatFix) -> np.ndarray:
        covariance = np.asarray(
            message.position_covariance, dtype=np.float64
        ).reshape((3, 3))
        if (
            message.position_covariance_type
            == NavSatFix.COVARIANCE_TYPE_UNKNOWN
            or not np.all(np.isfinite(covariance))
        ):
            return np.eye(3, dtype=np.float64) * (
                self.default_gnss_position_variance
            )

        covariance = 0.5 * (covariance + covariance.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        if np.min(eigenvalues) < -1e-6:
            return np.eye(3, dtype=np.float64) * (
                self.default_gnss_position_variance
            )
        eigenvalues = np.maximum(eigenvalues, 0.01)
        return eigenvectors @ np.diag(eigenvalues) @ eigenvectors.T

    def publish_pose(self) -> None:
        if (
            not self.filter.initialized
            or self.orientation is None
            or self.last_imu_stamp is None
        ):
            return

        message = Odometry()
        message.header.stamp = self.last_imu_stamp
        message.header.frame_id = self.world_frame
        message.child_frame_id = self.body_frame
        message.pose.pose.position.x = float(self.filter.state[0])
        message.pose.pose.position.y = float(self.filter.state[1])
        message.pose.pose.position.z = float(self.filter.state[2])
        (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        ) = (float(value) for value in self.orientation)
        inverse_orientation = self.orientation.copy()
        inverse_orientation[:3] *= -1.0
        linear_velocity = quaternion_rotate(
            inverse_orientation, self.filter.state[3:]
        )
        (
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
            message.twist.twist.linear.z,
        ) = (float(value) for value in linear_velocity)
        (
            message.twist.twist.angular.x,
            message.twist.twist.angular.y,
            message.twist.twist.angular.z,
        ) = (float(value) for value in self.angular_velocity)

        pose_covariance = np.zeros((6, 6), dtype=np.float64)
        pose_covariance[:3, :3] = self.filter.covariance[:3, :3]
        pose_covariance[3:, 3:] = self.covariance_matrix(
            self.orientation_covariance
        )
        twist_covariance = np.zeros((6, 6), dtype=np.float64)
        rotation = np.column_stack([
            quaternion_rotate(self.orientation, axis)
            for axis in np.eye(3, dtype=np.float64)
        ])
        world_to_body = rotation.T
        twist_covariance[:3, :3] = (
            world_to_body
            @ self.filter.covariance[3:, 3:]
            @ world_to_body.T
        )
        twist_covariance[3:, 3:] = self.covariance_matrix(
            self.angular_velocity_covariance
        )
        message.pose.covariance = pose_covariance.reshape(36).tolist()
        message.twist.covariance = twist_covariance.reshape(36).tolist()
        self.pose_publisher.publish(message)

    @staticmethod
    def covariance_matrix(values: np.ndarray) -> np.ndarray:
        if values.shape != (9,) or not np.all(np.isfinite(values)):
            return np.zeros((3, 3), dtype=np.float64)
        return values.reshape((3, 3))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotStateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
