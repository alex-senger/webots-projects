"""Webots device setup for e-puck controllers."""

from dataclasses import dataclass
from typing import Any

from epucklib.epuck import PS_NAMES


@dataclass
class EpuckDevices:
    """Everything a controller needs, already enabled and configured."""

    timestep: int
    sensors: list
    left_motor: Any
    right_motor: Any
    left_encoder: Any
    right_encoder: Any


def setup(robot, timestep: int | None = None) -> EpuckDevices:
    """Enable the proximity sensors and encoders, put both motors in velocity mode.

    `timestep` defaults to the world's basic time step; pass a multiple of it to
    run the control loop more slowly than the physics.
    """
    if timestep is None:
        timestep = int(robot.getBasicTimeStep())

    sensors = []
    for name in PS_NAMES:
        sensor = robot.getDevice(name)
        sensor.enable(timestep)
        sensors.append(sensor)

    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    for motor in (left_motor, right_motor):
        # Position control is the Webots default; an infinite target is how you
        # ask a motor for plain velocity control instead.
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)

    left_encoder = robot.getDevice("left wheel sensor")
    right_encoder = robot.getDevice("right wheel sensor")
    for encoder in (left_encoder, right_encoder):
        encoder.enable(timestep)

    return EpuckDevices(
        timestep=timestep,
        sensors=sensors,
        left_motor=left_motor,
        right_motor=right_motor,
        left_encoder=left_encoder,
        right_encoder=right_encoder,
    )
