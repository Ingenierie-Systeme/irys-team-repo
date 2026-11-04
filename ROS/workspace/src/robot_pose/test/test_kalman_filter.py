import numpy as np

from robot_pose.geometry import (
    geodetic_to_enu,
    pressure_to_depth,
    quaternion_rotate,
)
from robot_pose.kalman_filter import PositionVelocityFilter


def test_geodetic_origin_maps_to_zero_enu():
    position = geodetic_to_enu(48.39, -4.48, 0.0, 48.39, -4.48, 0.0)

    assert np.allclose(position, np.zeros(3))


def test_quaternion_rotation_uses_ros_xyzw_order():
    half_sqrt = np.sqrt(0.5)
    quaternion = np.array([0.0, 0.0, half_sqrt, half_sqrt])

    assert np.allclose(
        quaternion_rotate(quaternion, np.array([1.0, 0.0, 0.0])),
        [0.0, 1.0, 0.0],
    )


def test_pressure_is_converted_to_non_negative_depth():
    pressure_pa = 101325.0 + 1000.0 * 9.80665 * 2.0

    assert np.isclose(pressure_to_depth(pressure_pa, 101325.0, 1000.0), 2.0)
    assert pressure_to_depth(101000.0, 101325.0, 1000.0) == 0.0


def test_prediction_integrates_acceleration():
    estimator = PositionVelocityFilter(initial_velocity_variance=1.0)
    estimator.initialize(np.zeros(3), np.eye(3))

    estimator.predict(
        np.array([2.0, 0.0, 0.0]),
        delta_time=0.5,
        acceleration_noise_stddev=0.1,
    )

    assert np.allclose(estimator.state[:3], [0.25, 0.0, 0.0])
    assert np.allclose(estimator.state[3:], [1.0, 0.0, 0.0])


def test_gnss_update_corrects_prediction():
    estimator = PositionVelocityFilter(initial_velocity_variance=1.0)
    estimator.initialize(np.zeros(3), np.eye(3))
    estimator.predict(
        np.array([1.0, 0.0, 0.0]),
        delta_time=1.0,
        acceleration_noise_stddev=0.1,
    )
    predicted_x = estimator.state[0]

    accepted = estimator.update_position(
        np.array([0.0, 0.0, 0.0]),
        np.eye(3) * 0.01,
        mahalanobis_threshold=16.27,
    )

    assert accepted
    assert estimator.state[0] < predicted_x
    assert estimator.covariance[0, 0] > 0.0


def test_rejects_gnss_outlier():
    estimator = PositionVelocityFilter(initial_velocity_variance=1.0)
    estimator.initialize(np.zeros(3), np.eye(3) * 0.1)

    accepted = estimator.update_position(
        np.array([100.0, 0.0, 0.0]),
        np.eye(3) * 0.01,
        mahalanobis_threshold=16.27,
    )

    assert not accepted
    assert np.allclose(estimator.state[:3], 0.0)


def test_pressure_update_corrects_only_vertical_position():
    estimator = PositionVelocityFilter(initial_velocity_variance=1.0)
    estimator.initialize(np.zeros(3), np.eye(3))

    accepted = estimator.update_vertical_position(
        position_z=-2.0,
        measurement_variance=0.25,
        mahalanobis_threshold=16.27,
    )

    assert accepted
    assert np.allclose(estimator.state[:2], 0.0)
    assert estimator.state[2] < 0.0
    assert estimator.covariance[2, 2] < 1.0


def test_rejects_pressure_outlier():
    estimator = PositionVelocityFilter(initial_velocity_variance=1.0)
    estimator.initialize(np.zeros(3), np.eye(3) * 0.1)

    accepted = estimator.update_vertical_position(
        position_z=-100.0,
        measurement_variance=0.01,
        mahalanobis_threshold=16.27,
    )

    assert not accepted
    assert np.allclose(estimator.state[:3], 0.0)


def test_position_recalibration_preserves_velocity():
    estimator = PositionVelocityFilter(initial_velocity_variance=1.0)
    estimator.initialize(np.zeros(3), np.eye(3))
    estimator.state[3:] = [1.0, 2.0, 3.0]

    estimator.reset_position(np.array([4.0, 5.0, 6.0]), np.eye(3) * 0.1)

    assert np.allclose(estimator.state[:3], [4.0, 5.0, 6.0])
    assert np.allclose(estimator.state[3:], [1.0, 2.0, 3.0])
