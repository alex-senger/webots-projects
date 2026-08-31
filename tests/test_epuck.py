"""The physical constants must match the R2025a PROTO sources."""

from epucklib import epuck


def test_lookup_table_is_a_monotone_pair_of_equal_length_tuples():
    assert len(epuck.LOOKUP_DISTANCE_M) == len(epuck.LOOKUP_RAW)
    distances = epuck.LOOKUP_DISTANCE_M
    raws = epuck.LOOKUP_RAW
    # Distance increases while the raw reading falls off.
    assert all(a < b for a, b in zip(distances, distances[1:]))
    assert all(a > b for a, b in zip(raws, raws[1:]))


def test_lookup_table_endpoints_match_the_proto():
    assert epuck.LOOKUP_DISTANCE_M[0] == 0.0
    assert epuck.LOOKUP_RAW[0] == 4095.0
    assert epuck.LOOKUP_DISTANCE_M[-1] == 0.07
    assert epuck.LOOKUP_RAW[-1] == 67.19
    assert epuck.IR_MAX_RANGE_M == epuck.LOOKUP_DISTANCE_M[-1]


def test_eight_sensors_with_matching_offsets_and_angles():
    assert len(epuck.PS_NAMES) == 8
    assert len(epuck.SENSOR_OFFSETS_M) == 8
    assert len(epuck.SENSOR_ANGLES_RAD) == 8
    assert epuck.PS_NAMES[0] == "ps0"
    assert epuck.PS_NAMES[7] == "ps7"


def test_sensor_ring_is_left_right_symmetric():
    # ps0/ps7, ps1/ps6, ps2/ps5, ps3/ps4 are mirror pairs about the x axis.
    for right, left in ((0, 7), (1, 6), (2, 5), (3, 4)):
        rx, ry = epuck.SENSOR_OFFSETS_M[right]
        lx, ly = epuck.SENSOR_OFFSETS_M[left]
        assert rx == lx
        assert ry == -ly
        assert epuck.SENSOR_ANGLES_RAD[right] == -epuck.SENSOR_ANGLES_RAD[left]
        # Right-hand sensors sit at negative y (Webots: x forward, y left).
        assert ry <= 0.0


def test_geometry_uses_the_calibrated_track_width():
    assert epuck.WHEEL_RADIUS_M == 0.0205
    # Not the geometric 0.052 m: contact scrub makes the effective value larger.
    assert epuck.AXLE_LENGTH_M == 0.057
    assert epuck.BODY_RADIUS_M == 0.037
    assert epuck.MAX_WHEEL_SPEED == 6.28


def test_map_trust_range_is_inside_the_sensor_range():
    assert 0.0 < epuck.IR_MAP_TRUST_RANGE_M < epuck.IR_MAX_RANGE_M
