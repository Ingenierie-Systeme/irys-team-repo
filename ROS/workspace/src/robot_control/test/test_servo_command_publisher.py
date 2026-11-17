import math

import pytest

from robot_control.servo_command import (
    convert_direction_to_servo_positions,
    convert_rc_to_servo_positions,
)


def test_forward_direction_maps_to_neutral_fin_and_forward_mass() -> None:
    assert convert_direction_to_servo_positions(1.0, 0.0, 0.0) == (0.0, 50.0)


def test_diagonal_direction_maps_to_heading_and_mass_angle() -> None:
    fin_angle, mass_angle = convert_direction_to_servo_positions(
        0.5,
        math.sqrt(3.0) / 2.0,
        0.0,
    )

    assert fin_angle == pytest.approx(60.0)
    assert mass_angle == pytest.approx(25.0)


def test_fin_oscillation_scales_with_direction_magnitude() -> None:
    fin_angle, mass_angle = convert_direction_to_servo_positions(
        0.5,
        0.0,
        1.25,
    )

    assert fin_angle == pytest.approx(15.0)
    assert mass_angle == pytest.approx(25.0)


def test_fin_angle_is_limited_to_servo_range() -> None:
    upper_angle, _ = convert_direction_to_servo_positions(
        0.5,
        math.sqrt(3.0) / 2.0,
        0.75,
    )
    lower_angle, _ = convert_direction_to_servo_positions(
        0.5,
        math.sqrt(3.0) / 2.0,
        2.25,
    )

    assert upper_angle == pytest.approx(85.0)
    assert lower_angle == pytest.approx(35.0)


def test_zero_direction_centers_both_servos() -> None:
    assert convert_direction_to_servo_positions(0.0, 0.0, 1.25) == (0.0, 0.0)


def test_mass_angle_is_limited_to_servo_range() -> None:
    _, mass_angle = convert_direction_to_servo_positions(2.0, 0.0, 0.0)

    assert mass_angle == 50.0


def test_rc_throttle_controls_fin_and_aileron_controls_mass() -> None:
    fin_angle, mass_angle = convert_rc_to_servo_positions(
        1.0,
        -1.0,
        0.75,
    )

    assert fin_angle == pytest.approx(60.0)
    assert mass_angle == -50.0


def test_rc_channels_are_clamped_to_servo_limits() -> None:
    fin_angle, mass_angle = convert_rc_to_servo_positions(
        2.0,
        -2.0,
        0.75,
    )

    assert fin_angle == pytest.approx(60.0)
    assert mass_angle == -50.0


def test_zero_throttle_does_not_oscillate_fin() -> None:
    fin_angle, mass_angle = convert_rc_to_servo_positions(
        0.0,
        0.5,
        0.75,
    )

    assert fin_angle == 0.0
    assert mass_angle == 25.0
