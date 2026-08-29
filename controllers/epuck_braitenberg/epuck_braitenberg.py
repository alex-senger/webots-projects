"""e-puck obstacle avoidance by Braitenberg sensor-motor coupling.

There is no internal state and no explicit decision anywhere in this file.
Each proximity sensor is wired directly to both wheels through a fixed
weight; avoidance is an emergent consequence of that wiring, in the sense
described by Braitenberg (1984).

The weight matrix and the normalisation are those of the official Webots
sample controller `e-puck_avoid_obstacles`, so the behaviour is a faithful
Python reimplementation rather than an invention.
"""
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from controller import Robot

from epucklib import devices, epuck, odometry, trace

CONTROLLER_NAME = "epuck_braitenberg"

SENSOR_MAX_RAW = 4096.0
TRACE_SECONDS = 60.0
FIELDNAMES = ["t_s", "mode", "front_max_raw", "v_left", "v_right", "distance_m"]
# ps0 and ps7 straddle the nose of the robot.
FRONT = (0, 7)

# One (left, right) pair per sensor, ps0 first. Negative weights make a
# wheel slow or reverse as that sensor lights up, which is what turns the
# robot away from an obstacle.
WEIGHTS = (
    (-1.3, -1.0),
    (-1.3, -1.0),
    (-0.5, 0.5),
    (0.0, 0.0),
    (0.0, 0.0),
    (0.05, -0.5),
    (-0.75, 0.0),
    (-0.75, 0.0),
)

# Baseline forward drive that the sensor terms modulate.
OFFSETS = (0.5 * epuck.MAX_WHEEL_SPEED, 0.5 * epuck.MAX_WHEEL_SPEED)


def main():
    robot = Robot()
    dev = devices.setup(robot)
    timestep = dev.timestep
    dt_s = timestep / 1000.0

    writer = trace.CsvTrace(trace.trace_path(CONTROLLER_NAME, REPO_ROOT), FIELDNAMES)
    distance_m = 0.0

    while robot.step(timestep) != -1:
        readings = [sensor.getValue() for sensor in dev.sensors]
        normalised = [value / SENSOR_MAX_RAW for value in readings]

        speeds = []
        for wheel in (0, 1):
            coupling = sum(
                normalised[i] * WEIGHTS[i][wheel] for i in range(len(readings))
            )
            speeds.append(
                devices.clamp_speed(OFFSETS[wheel] + coupling * epuck.MAX_WHEEL_SPEED)
            )

        dev.left_motor.setVelocity(speeds[0])
        dev.right_motor.setVelocity(speeds[1])

        distance_m += odometry.integrate_distance(speeds[0], speeds[1], dt_s)
        if robot.getTime() <= TRACE_SECONDS:
            writer.write(
                t_s=f"{robot.getTime():.4f}",
                mode="BRAITENBERG",
                front_max_raw=f"{max(readings[i] for i in FRONT):.2f}",
                v_left=f"{speeds[0]:.4f}",
                v_right=f"{speeds[1]:.4f}",
                distance_m=f"{distance_m:.5f}",
            )
        elif not writer.closed:
            writer.close()


if __name__ == "__main__":
    main()
