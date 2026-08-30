# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified from the Cyberbotics `scara_food_industry` sample.

"""SCARA fruit sorting with the two bins deliberately swapped.

1. The robot alternates orange, apple, orange, ... and places each fruit in
   the crate originally intended for the other one.
2. The LED toggles on every fifth orange that has been picked *and* placed,
   so it changes state after oranges 5, 10, 15, ...

The grasp is faked the way the original sample does it: with supervisor access
the fruit node is teleported just under the suction tool on every step it is
held, and released by simply no longer doing that.
"""

from controller import Supervisor

# The world DEFs fruit1 as the Orange and fruit0 as the Apple; the number is
# the suffix of the DEF name, so it doubles as the node lookup key.
ORANGE = 1
APPLE = 0
FRUIT_NAME = {ORANGE: "orange", APPLE: "apple"}

# (base_arm, arm) joint targets for the two crates, each named for the fruit it
# was originally meant to receive.
ORANGE_BIN = (0.0, -0.83)
APPLE_BIN = (-0.50, -0.83)

# Requirement 1: every fruit goes to the *other* fruit's crate.
TARGET_BIN = {ORANGE: APPLE_BIN, APPLE: ORANGE_BIN}
BIN_NAME = {ORANGE: "apple", APPLE: "orange"}

# Requirement 2.
ORANGES_PER_TOGGLE = 5

PICK_POSE = (0.2, 0.6)  # (base_arm, arm) over the incoming fruit
SHAFT_DOWN = -0.148
SHAFT_UP = 0.0
GRASP_OFFSET_M = 0.07  # how far below the tool a held fruit hangs

# Step counts within one pick-and-place cycle, carried over from the sample.
# Targets latch, so a phase only has to be commanded on the step it begins;
# only the grasp needs repeating, because it teleports the fruit each step.
LOWER_AT = 55
GRASP_AT = 75
LIFT_AT = 90
TRAVEL_AT = 125
RELEASE_AT = 200
CYCLE_END = 276


class Led:
    """The epson_led, kept in a known state so its changes carry meaning."""

    def __init__(self, device) -> None:
        self.device = device
        self.on = False
        self.device.set(0)

    def toggle(self) -> bool:
        self.on = not self.on
        self.device.set(1 if self.on else 0)
        return self.on


robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

base_arm = robot.getDevice("base_arm_motor")
arm = robot.getDevice("arm_motor")
shaft = robot.getDevice("shaft_linear_motor")
led = Led(robot.getDevice("epson_led"))
vacuum = robot.getFromDef("VACCUM")


def hold(fruit_type: int) -> None:
    """Keep a fruit stuck to the suction tool for this step."""
    fruit = robot.getFromDef(f"fruit{fruit_type}")
    if fruit is None:
        return
    x, y, z = vacuum.getPosition()
    fruit.getField("translation").setSFVec3f([x, y, z - GRASP_OFFSET_M])
    fruit.resetPhysics()


fruit = ORANGE
oranges_placed = 0
cycle = 1
step = 0

while robot.step(timestep) != -1:
    if step == 0:
        base_arm.setPosition(PICK_POSE[0])
        arm.setPosition(PICK_POSE[1])

    elif step == LOWER_AT:
        shaft.setPosition(SHAFT_DOWN)

    elif GRASP_AT <= step < RELEASE_AT:
        hold(fruit)
        if step == LIFT_AT:
            shaft.setPosition(SHAFT_UP)
        elif step == TRAVEL_AT:
            base_target, arm_target = TARGET_BIN[fruit]
            base_arm.setPosition(base_target)
            arm.setPosition(arm_target)

    elif step == RELEASE_AT:
        # The fruit stops being carried here, which is what drops it in the
        # crate -- so this is the step on which a place actually completes.
        report = (
            f"cycle {cycle:3d}  placed {FRUIT_NAME[fruit]:6s} "
            f"in the {BIN_NAME[fruit]:6s} bin"
        )
        if fruit == ORANGE:
            oranges_placed += 1
            report += f"  (orange #{oranges_placed})"
            if oranges_placed % ORANGES_PER_TOGGLE == 0:
                report += f"  -- LED {'ON' if led.toggle() else 'OFF'}"
        print(report, flush=True)

    elif step == CYCLE_END:
        fruit = APPLE if fruit == ORANGE else ORANGE
        cycle += 1
        step = -1

    step += 1
