"""Turning e-puck infra-red readings into geometry.

The e-puck's proximity sensors report a raw number, not a distance. Rather
than picking magic thresholds, this module inverts the very lookup table the
simulator uses to produce those numbers, so a reading becomes a distance in
metres and every downstream decision can be expressed in real units.
"""

import math
from collections.abc import Sequence

import numpy as np

from epucklib.epuck import (
    FRONT_SENSORS,
    IR_MAX_RANGE_M,
    LOOKUP_DISTANCE_M,
    LOOKUP_RAW,
    SENSOR_ANGLES_RAD,
    SENSOR_OFFSETS_M,
)

# epucklib.odometry imports only control and epuck, so this is a leaf-ward
# import: no cycle.
from epucklib.odometry import Pose

# np.interp needs its sample points increasing, and the raw value *falls* with
# distance, so both table columns are reversed once here at import time.
_RAW_ASCENDING = np.array(LOOKUP_RAW[::-1])
_DISTANCE_FOR_RAW = np.array(LOOKUP_DISTANCE_M[::-1])


def raw_to_distance(raw: float) -> float:
    """Distance in metres for one raw proximity reading.

    Readings at or below the table's weakest entry mean 'nothing within range'
    and saturate at IR_MAX_RANGE_M; readings at or above the strongest entry
    mean contact and saturate at zero.
    """
    return float(np.interp(raw, _RAW_ASCENDING, _DISTANCE_FOR_RAW))


def read_distances(raw_readings: Sequence[float]) -> list[float]:
    """Convert a whole ring of raw readings to metres."""
    return [raw_to_distance(value) for value in raw_readings]


def sensor_ray(
    pose: Pose, index: int, distance_m: float
) -> tuple[tuple[float, float], tuple[float, float]]:
    """World-frame (origin, endpoint) of one sensor's ray.

    The origin is where the sensor actually sits on the robot's shell, not the
    robot's centre; over a 2 cm grid that 3 cm offset is the difference between
    marking the right cell and the one behind it.
    """
    offset_x, offset_y = SENSOR_OFFSETS_M[index]
    cos_t, sin_t = math.cos(pose.theta), math.sin(pose.theta)
    origin_x = pose.x + cos_t * offset_x - sin_t * offset_y
    origin_y = pose.y + sin_t * offset_x + cos_t * offset_y

    ray_angle = pose.theta + SENSOR_ANGLES_RAD[index]
    end_x = origin_x + distance_m * math.cos(ray_angle)
    end_y = origin_y + distance_m * math.sin(ray_angle)
    return (origin_x, origin_y), (end_x, end_y)


def repulsion(
    distances: Sequence[float], tangent_weight: float = 0.8
) -> tuple[float, float, float]:
    """Repulsive field in the robot frame, plus a front-proximity factor.

    Each sensor in range pushes back along its own axis, strength rising from
    0 at maximum range to 1 at contact. The four forward sensors add a
    tangential "vortex" term -- the radial push turned 90 degrees away from
    that sensor's side -- so the robot slides around an obstacle rather than
    oscillating in front of it, the classic cure for the head-on local minimum.

    A symmetric head-on obstacle still cancels the tangential terms; the caller
    breaks that tie (see `blend_command`).
    """
    force_x = force_y = front = 0.0
    for index, distance in enumerate(distances):
        if distance >= IR_MAX_RANGE_M:
            continue
        strength = 1.0 - distance / IR_MAX_RANGE_M
        angle = SENSOR_ANGLES_RAD[index]
        radial_x, radial_y = -math.cos(angle), -math.sin(angle)
        force_x += strength * radial_x
        force_y += strength * radial_y

        if index in FRONT_SENSORS:
            if angle >= 0.0:
                tangent_x, tangent_y = -radial_y, radial_x
            else:
                tangent_x, tangent_y = radial_y, -radial_x
            force_x += strength * tangent_weight * tangent_x
            force_y += strength * tangent_weight * tangent_y
            front = max(front, strength)

    return force_x, force_y, front
