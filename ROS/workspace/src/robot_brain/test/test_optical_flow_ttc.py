import numpy as np

import pytest

from robot_brain.optical_flow_ttc import (
    choose_safe_direction,
    estimate_sector_time_to_contact,
    estimate_time_to_contact,
)


def test_estimate_ttc_from_radial_expansion():
    height, width = 60, 80
    rate = 2.0
    x = np.arange(width, dtype=np.float32) - (width - 1) / 2.0
    y = np.arange(height, dtype=np.float32) - (height - 1) / 2.0
    grid_x, grid_y = np.meshgrid(x, y)
    flow = np.stack((grid_x * rate, grid_y * rate), axis=-1)

    assert estimate_time_to_contact(flow) == pytest.approx(1.0 / rate)


def test_no_expansion_has_no_ttc():
    flow = np.zeros((60, 80, 2), dtype=np.float32)

    assert estimate_time_to_contact(flow) is None


def test_sector_ttc_detects_expansion_on_left():
    height, width = 60, 90
    center_x = (width - 1) / 2.0
    x = (np.arange(width, dtype=np.float32) - center_x) / (width / 2.0)
    y = (
        np.arange(height, dtype=np.float32) - (height - 1) / 2.0
    ) / (height / 2.0)
    flow_x = np.zeros((height, width), dtype=np.float32)
    flow_y = np.zeros_like(flow_x)
    left = x < -0.8 / 3.0
    flow_x[:, left] = x[left] * 4.0 * (width / 2.0)
    flow_y[:, left] = y[:, None] * 4.0 * (height / 2.0)
    flow = np.stack((flow_x, flow_y), axis=-1)

    left_ttc, center_ttc, right_ttc = estimate_sector_time_to_contact(flow)

    assert left_ttc == pytest.approx(0.25)
    assert center_ttc is None
    assert right_ttc is None


def test_direction_keeps_course_when_center_is_clear():
    assert choose_safe_direction((None, 3.0, 1.0), 2.0) == (1.0, 0.0, 0.0)


def test_direction_turns_towards_clearer_side():
    assert choose_safe_direction((3.0, 1.0, 5.0), 2.0) == (
        0.5,
        -0.8660254037844386,
        0.0,
    )


def test_direction_stops_when_all_sectors_are_unsafe():
    assert choose_safe_direction((1.0, 0.5, 1.5), 2.0) == (
        0.0,
        0.0,
        0.0,
    )
