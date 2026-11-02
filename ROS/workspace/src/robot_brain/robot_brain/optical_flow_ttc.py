"""Estimation du temps avant collision à partir d'un flux optique radial."""

import numpy as np


MIN_EXPANSION_RATE = 0.01
MIN_VALID_PIXELS = 12


def _normalized_expansion_rates(flow: np.ndarray) -> tuple[
    np.ndarray, np.ndarray, np.ndarray
]:
    if flow.ndim != 3 or flow.shape[2] != 2:
        raise ValueError('flow doit avoir la forme (hauteur, largeur, 2).')

    height, width, _ = flow.shape
    if height < 2 or width < 3:
        raise ValueError('flow doit mesurer au moins 2x3 pixels.')

    center_x = (width - 1) / 2.0
    center_y = (height - 1) / 2.0
    half_width = width / 2.0
    half_height = height / 2.0

    x = (np.arange(width, dtype=np.float32) - center_x) / half_width
    y = (np.arange(height, dtype=np.float32) - center_y) / half_height
    normalized_x, normalized_y = np.meshgrid(x, y)
    normalized_flow_x = flow[:, :, 0] / half_width
    normalized_flow_y = flow[:, :, 1] / half_height
    radius_squared = normalized_x**2 + normalized_y**2

    expansion_rate = np.zeros_like(radius_squared)
    np.divide(
        normalized_x * normalized_flow_x + normalized_y * normalized_flow_y,
        radius_squared,
        out=expansion_rate,
        where=radius_squared > 1e-6,
    )

    return normalized_x, normalized_y, expansion_rate


def estimate_time_to_contact(flow: np.ndarray) -> float | None:
    """Estime le TTC global en secondes à partir d'une expansion radiale."""
    normalized_x, normalized_y, expansion_rate = _normalized_expansion_rates(
        flow
    )
    roi = (
        (np.abs(normalized_x) <= 0.8)
        & (np.abs(normalized_y) <= 0.8)
        & np.isfinite(expansion_rate)
    )
    rates = expansion_rate[roi]
    rates = rates[
        np.isfinite(rates) & (rates > MIN_EXPANSION_RATE)
    ]
    if rates.size < max(MIN_VALID_PIXELS, int(roi.size * 0.001)):
        return None

    return 1.0 / float(np.percentile(rates, 75))


def estimate_sector_time_to_contact(
    flow: np.ndarray,
) -> tuple[float | None, float | None, float | None]:
    """Estime les TTC heuristiques des secteurs gauche, centre et droit."""
    normalized_x, normalized_y, expansion_rate = _normalized_expansion_rates(
        flow
    )
    vertical_roi = np.abs(normalized_y) <= 0.8
    sectors = (
        (normalized_x >= -0.8) & (normalized_x < -0.8 / 3.0),
        (normalized_x >= -0.8 / 3.0)
        & (normalized_x <= 0.8 / 3.0),
        (normalized_x > 0.8 / 3.0) & (normalized_x <= 0.8),
    )
    time_to_contact: list[float | None] = []
    for sector in sectors:
        roi = sector & vertical_roi
        rates = expansion_rate[roi]
        rates = rates[
            np.isfinite(rates) & (rates > MIN_EXPANSION_RATE)
        ]
        minimum_pixels = max(MIN_VALID_PIXELS, int(roi.size * 0.001))
        if rates.size < minimum_pixels:
            time_to_contact.append(None)
        else:
            time_to_contact.append(
                1.0 / float(np.percentile(rates, 75))
            )

    return tuple(time_to_contact)


def choose_safe_direction(
    sector_time_to_contact: tuple[
        float | None, float | None, float | None
    ],
    minimum_time_to_contact: float,
) -> tuple[float, float, float]:
    """Choisit avant ou en diagonale vers le côté le moins menaçant."""
    left_ttc, center_ttc, right_ttc = sector_time_to_contact
    if center_ttc is None or center_ttc >= minimum_time_to_contact:
        return (1.0, 0.0, 0.0)

    safe_sides = [
        (left_ttc, 1.0),
        (right_ttc, -1.0),
    ]
    safe_sides = [
        (ttc, side)
        for ttc, side in safe_sides
        if ttc is None or ttc >= minimum_time_to_contact
    ]
    if not safe_sides:
        return (0.0, 0.0, 0.0)

    ttc, side = max(
        safe_sides,
        key=lambda candidate: (
            float('inf') if candidate[0] is None else candidate[0]
        ),
    )
    return (0.5, side * 0.8660254037844386, 0.0)
