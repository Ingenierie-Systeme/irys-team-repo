"""Règles indépendantes de ROS pour l'arbitrage des consignes."""


Direction = tuple[float, float, float]
STOP_DIRECTION: Direction = (0.0, 0.0, 0.0)


def apply_safety_gate(
    direction: Direction,
    state_ready: bool,
    emergency_stop: bool,
) -> Direction:
    """Interdit tout déplacement si l'état n'est pas prêt ou si arrêt."""
    if not state_ready or emergency_stop:
        return STOP_DIRECTION
    return direction
