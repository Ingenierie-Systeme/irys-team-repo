import numpy as np

import pytest

from robot_brain.optical_flow_ttc import estimate_time_to_contact


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
