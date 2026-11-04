from robot_brain.brain_logic import (
    STOP_DIRECTION,
    apply_safety_gate,
)


def test_safety_gate_passes_direction_with_valid_state() -> None:
    direction = (0.5, 0.866, 0.0)

    assert apply_safety_gate(direction, True, False) == direction


def test_safety_gate_stops_without_valid_state() -> None:
    assert apply_safety_gate((1.0, 0.0, 0.0), False, False) == STOP_DIRECTION


def test_safety_gate_stops_when_emergency_stop_is_active() -> None:
    assert apply_safety_gate((1.0, 0.0, 0.0), True, True) == STOP_DIRECTION
