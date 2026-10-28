"""Filtre de Kalman linéaire pour la position et la vitesse en ENU."""

import numpy as np


class PositionVelocityFilter:
    """Estime position et vitesse à partir d'accélérations et de fixes GNSS."""

    def __init__(self, initial_velocity_variance: float) -> None:
        self.state = np.zeros(6, dtype=np.float64)
        self.covariance = np.zeros((6, 6), dtype=np.float64)
        self.initial_velocity_variance = initial_velocity_variance
        self.initialized = False

    def initialize(
        self,
        position: np.ndarray,
        position_covariance: np.ndarray,
    ) -> None:
        self.state[:3] = position
        self.state[3:] = 0.0
        self.covariance.fill(0.0)
        self.covariance[:3, :3] = position_covariance
        self.covariance[3:, 3:] = (
            np.eye(3, dtype=np.float64) * self.initial_velocity_variance
        )
        self.initialized = True

    def predict(
        self,
        acceleration: np.ndarray,
        delta_time: float,
        acceleration_noise_stddev: float,
    ) -> None:
        if not self.initialized or delta_time <= 0.0:
            return

        transition = np.eye(6, dtype=np.float64)
        transition[:3, 3:] = np.eye(3, dtype=np.float64) * delta_time
        control = np.zeros((6, 3), dtype=np.float64)
        control[:3, :] = np.eye(3, dtype=np.float64) * (0.5 * delta_time**2)
        control[3:, :] = np.eye(3, dtype=np.float64) * delta_time

        noise_variance = acceleration_noise_stddev**2
        process_noise = np.zeros((6, 6), dtype=np.float64)
        identity = np.eye(3, dtype=np.float64)
        process_noise[:3, :3] = identity * (delta_time**4 / 4.0)
        process_noise[:3, 3:] = identity * (delta_time**3 / 2.0)
        process_noise[3:, :3] = identity * (delta_time**3 / 2.0)
        process_noise[3:, 3:] = identity * delta_time**2
        process_noise *= noise_variance

        self.state = transition @ self.state + control @ acceleration
        self.covariance = (
            transition @ self.covariance @ transition.T + process_noise
        )
        self.covariance = 0.5 * (self.covariance + self.covariance.T)

    def reset_position(
        self,
        position: np.ndarray,
        position_covariance: np.ndarray,
    ) -> None:
        """Recalibre la position sans réinitialiser la vitesse estimée."""
        self.state[:3] = position
        self.covariance[:3, :3] = position_covariance
        self.covariance[:3, 3:] = 0.0
        self.covariance[3:, :3] = 0.0

    def update_position(
        self,
        position: np.ndarray,
        measurement_covariance: np.ndarray,
        mahalanobis_threshold: float,
    ) -> bool:
        if not self.initialized:
            self.initialize(position, measurement_covariance)
            return True

        observation = np.zeros((3, 6), dtype=np.float64)
        observation[:, :3] = np.eye(3, dtype=np.float64)
        innovation = position - observation @ self.state
        innovation_covariance = (
            observation @ self.covariance @ observation.T
            + measurement_covariance
        )

        try:
            normalized_innovation = np.linalg.solve(
                innovation_covariance, innovation
            )
        except np.linalg.LinAlgError:
            return False

        distance_squared = float(innovation @ normalized_innovation)
        if (
            not np.isfinite(distance_squared)
            or distance_squared > mahalanobis_threshold
        ):
            return False

        gain = np.linalg.solve(
            innovation_covariance,
            observation @ self.covariance,
        ).T
        self.state += gain @ innovation

        residual_projection = np.eye(6, dtype=np.float64) - gain @ observation
        self.covariance = (
            residual_projection
            @ self.covariance
            @ residual_projection.T
            + gain @ measurement_covariance @ gain.T
        )
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        return True
