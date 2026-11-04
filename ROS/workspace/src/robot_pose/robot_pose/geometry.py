"""Conversions géométriques utilisées par l'estimateur de pose."""

import math

import numpy as np


EARTH_RADIUS_METERS = 6_371_000.0
GRAVITY_METERS_PER_SECOND_SQUARED = 9.80665


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


def pressure_to_depth(
    pressure_pa: float,
    surface_pressure_pa: float,
    fluid_density_kg_m3: float,
) -> float:
    """Convert absolute fluid pressure to non-negative depth in meters."""
    pressure_difference = max(0.0, pressure_pa - surface_pressure_pa)
    return pressure_difference / (
        fluid_density_kg_m3 * GRAVITY_METERS_PER_SECOND_SQUARED
    )
