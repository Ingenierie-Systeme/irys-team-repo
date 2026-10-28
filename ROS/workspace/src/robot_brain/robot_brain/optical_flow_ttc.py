"""Estimation du temps avant collision à partir d'un flux optique radial."""

import numpy as np


MIN_EXPANSION_RATE = 0.01
MIN_VALID_PIXELS = 12


def estimate_time_to_contact(flow: np.ndarray) -> float | None:
    """Estime le TTC en secondes, ou retourne None sans expansion visible.

    Le calcul suppose que le point de fuite est au centre de l'image et que
    l'expansion observée provient d'un mouvement vers un obstacle fixe.
    """
    height, width, _ = flow.shape
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

    roi = (
        (np.abs(normalized_x) <= 0.8)
        & (np.abs(normalized_y) <= 0.8)
        & (expansion_rate > MIN_EXPANSION_RATE)
        & np.isfinite(expansion_rate)
    )
    rates = expansion_rate[roi]
    if rates.size < max(MIN_VALID_PIXELS, int(roi.size * 0.001)):
        return None

    representative_rate = float(np.percentile(rates, 75))
    return 1.0 / representative_rate
