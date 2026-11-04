"""Tests for the Navigator node servo output callback."""

import importlib
import sys
from types import ModuleType, SimpleNamespace

import pytest

# The servo callback can be tested without the Navigator board SDK. Supply an
# import stub when that optional hardware package is not installed.
try:
    import bluerobotics_navigator  # noqa: F401
except ImportError:
    sys.modules["bluerobotics_navigator"] = ModuleType(
        "bluerobotics_navigator"
    )

node_navigator = importlib.import_module("navigator.node_navigator")
from navigator.config import DUTY_MAX, DUTY_MID


@pytest.fixture
def servo_node(monkeypatch):
    """Create a node instance without initializing the physical board."""
    calls = []
    monkeypatch.setattr(
        node_navigator.navigator,
        "set_pwm_channel_duty_cycle",
        lambda channel, duty: calls.append((channel, duty)),
        raising=False,
    )

    # __init__ initializes the attached Navigator board, so exercise the
    # callback on an uninitialized instance and mock only its hardware output.
    node = node_navigator.NavigatorNode.__new__(node_navigator.NavigatorNode)
    return node, calls


def test_pwm_callback_maps_normalized_commands_to_servos(servo_node):
    node, calls = servo_node

    node.pwm_callback(SimpleNamespace(data=[0.0, 1.0]))

    assert calls == [
        (1, DUTY_MID),
        (0, DUTY_MAX),
    ]


@pytest.mark.parametrize(
    "command, expected_duty",
    [
        (-2.0, 2 * DUTY_MID - DUTY_MAX),
        (2.0, DUTY_MAX),
    ],
)
def test_pwm_callback_clips_commands_to_normalized_range(
    servo_node, command, expected_duty
):
    node, calls = servo_node

    node.pwm_callback(SimpleNamespace(data=[command, command]))

    assert calls == [
        (1, expected_duty),
        (0, expected_duty),
    ]


def test_pwm_callback_maps_negative_full_scale(servo_node):
    node, calls = servo_node

    node.pwm_callback(SimpleNamespace(data=[-1.0, -1.0]))

    expected_duty = 2 * DUTY_MID - DUTY_MAX
    assert calls == [(1, expected_duty), (0, expected_duty)]
