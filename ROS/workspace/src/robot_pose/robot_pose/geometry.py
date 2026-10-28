"""Conversions géométriques utilisées par l'estimateur de pose."""

import math

import numpy as np


EARTH_RADIUS_METERS = 6_371_000.0


def quaternion_rotate(
    quaternion: np.ndarray,
    vector: np.ndarray,
) -> np.ndarray:
    """Applique une rotation xyzw à un vecteur."""
    vector_part = quaternion[:3]
    cross = 2.0 * np.cross(vector_part, vector)
    return vector + quaternion[3] * cross + np.cross(vector_part, cross)


def geodetic_to_enu(
    latitude: float,
    longitude: float,
    altitude: float,
    origin_latitude: float,
    origin_longitude: float,
    origin_altitude: float,
) -> np.ndarray:
    """Projette une position géodésique en coordonnées locales ENU."""
    latitude_radians = math.radians(latitude)
    longitude_radians = math.radians(longitude)
    origin_latitude_radians = math.radians(origin_latitude)
    origin_longitude_radians = math.radians(origin_longitude)

    east = (
        EARTH_RADIUS_METERS
        * math.cos(origin_latitude_radians)
        * (longitude_radians - origin_longitude_radians)
    )
    north = EARTH_RADIUS_METERS * (
        latitude_radians - origin_latitude_radians
    )
    up = altitude - origin_altitude
    return np.array([east, north, up], dtype=np.float64)
