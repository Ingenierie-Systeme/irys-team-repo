import math


FIN_SWIM_AMPLITUDE_DEGREES = 40.0
FIN_SWIM_PERIOD_SECONDS = 5.0
FIN_MAX_ANGLE_DEGREES = 90.0
MASS_MAX_ANGLE_DEGREES = 50.0


def convert_direction_to_servo_positions(
    direction_x: float,
    direction_y: float,
    elapsed_seconds: float,
) -> tuple[float, float]:
    """Convert a direction vector into fin and mobile-mass servo angles."""
    speed = min(math.hypot(direction_x, direction_y), 1.0)
    if speed == 0.0:
        heading_degrees = 0.0
    else:
        heading_degrees = max(
            -FIN_MAX_ANGLE_DEGREES,
            min(
                math.degrees(math.atan2(direction_y, direction_x)),
                FIN_MAX_ANGLE_DEGREES,
            ),
        )

    oscillation_amplitude = min(
        FIN_SWIM_AMPLITUDE_DEGREES * speed,
        FIN_MAX_ANGLE_DEGREES - abs(heading_degrees),
    )
    fin_oscillation = (
        oscillation_amplitude
        * math.sin(2.0 * math.pi * elapsed_seconds / FIN_SWIM_PERIOD_SECONDS)
    )
    fin_angle = heading_degrees + fin_oscillation
    mass_angle = max(-1.0, min(direction_x, 1.0)) * MASS_MAX_ANGLE_DEGREES
    return fin_angle, mass_angle
