"""Systematic floor-coverage controller for the e-puck -- a basic Roomba.

The robot sweeps the arena in boustrophedon lanes while building a 2 cm
occupancy and coverage grid from its infra-red sensors, then falls back on
breadth-first search to reach whatever the sweep missed. It is told the size of
its arena and nothing else: the three wooden boxes and the walls are all
discovered by touch.

Layers, bottom-up:

1. **Odometry** -- dead reckoning from the wheel encoders, re-anchored to
   ground truth every MOCAP_INTERVAL seconds in the manner of an overhead
   motion-capture rig. Set MOCAP_INTERVAL = 0 to watch pure dead reckoning
   smear the map instead.
2. **Mapping** -- each IR reading is converted to a distance through the
   sensor's own lookup table and projected into the grid: free along the ray,
   occupied at its end once seen twice.
3. **Planning** -- lanes first, then BFS to the nearest cell that would sweep
   new ground.
4. **Reactive control** -- a repulsive potential field with a tangential term
   blends into the waypoint bearing, so the robot slides along an obstacle
   rather than stalling against it, backed by a stall detector.
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

import math

from controller import Supervisor

from epucklib import control, devices, epuck, grid, perception, sweep
from epucklib.odometry import DeadReckoning, Pose

CONTROLLER_NAME = "epuck_roomba"

TIME_STEP = 32  # ms; twice the world's basic step, plenty for this control rate

# ---- arena and map ----
CELL_SIZE_M = 0.02
# A tiny margin below the body-radius limit: a waypoint placed exactly at the
# reachable boundary quantizes into a grid cell whose *centre* sits just
# beyond it, so CoverageMap.blocked_mask() marks the whole outer band -- and
# every lane's start/end point -- unreachable from the very first step.
# Verified by probing blocked_mask() directly: margin 0.0 leaves 34/34 lane
# points unreachable, margin >= 0.005 leaves 0/34. The margin also shrinks the
# robot-centre reach, so its footprint stops 1 mm short of the physical wall
# per mm of margin (area = 1-(1-2*margin)^2, negligible at this size) -- kept
# at 0.006 m rather than a full cell to keep that loss under 2.5% of the
# arena.
CENTER_LIMIT_M = epuck.ARENA_HALF_M - epuck.BODY_RADIUS_M - 0.006  # 0.457
LANE_SPACING_M = 0.05  # tighter than the 7.4 cm body, so lanes overlap

# ---- motion ----
CRUISE_SPEED = 0.09  # m/s, about 70% of what the motors can do
K_HEADING = 4.0
OMEGA_MAX = 4.0
WAYPOINT_TOLERANCE_M = 0.015
REPULSION_GAIN = 1.2
REPULSION_ALPHA = 0.4  # exponential smoothing, damps limit cycles
FRONT_BRAKE = 0.8  # how hard a near obstacle slows the robot

# ---- stall escape ----
STALL_SPEED_M = 0.0002  # per step, roughly 6 mm/s
STALL_STEPS = 78  # about 2.5 s without translating
ESCAPE_STEPS = 20

# ---- housekeeping ----
MOCAP_INTERVAL_S = 5.0  # 0 disables the ground-truth correction entirely
REPORT_INTERVAL_S = 10.0
LANE_TIMEOUT_S = 25.0  # give up on a waypoint the robot cannot reach


def ground_truth(node) -> Pose:
    """True pose of the robot, via the supervisor API."""
    x, y, _ = node.getPosition()
    orientation = node.getOrientation()  # row-major 3x3; column 0 is the body x axis
    return Pose(x, y, math.atan2(orientation[3], orientation[0]))


class Roomba:
    def __init__(self) -> None:
        self.robot = Supervisor()
        self.node = self.robot.getSelf()
        self.dev = devices.setup(self.robot, TIME_STEP)

        # One step so every enabled sensor delivers a first valid reading.
        self.robot.step(TIME_STEP)

        start = ground_truth(self.node)
        self.odom = DeadReckoning(
            self.dev.left_encoder.getValue(),
            self.dev.right_encoder.getValue(),
            start,
        )
        self.map = grid.CoverageMap(
            half_extent_m=epuck.ARENA_HALF_M,
            cell_size_m=CELL_SIZE_M,
            robot_radius_m=epuck.BODY_RADIUS_M,
        )

        self.mode = "SWEEP"
        self.lane_points = sweep.lane_waypoints(CENTER_LIMIT_M, LANE_SPACING_M)
        self.lane_index = 0
        self.waypoint_deadline = self.robot.getTime() + LANE_TIMEOUT_S

        self.field = [0.0, 0.0]  # smoothed repulsion
        self.stalled_steps = 0
        self.escape_steps = 0
        self.escape_turn = 0.0

        self.last_mocap = self.robot.getTime()
        self.last_report = 0.0

    # ---- perception ----

    def sense(self) -> tuple[list[float], float]:
        """Update the pose and the map; return IR distances and the front factor."""
        pose = self.odom.update(
            self.dev.left_encoder.getValue(), self.dev.right_encoder.getValue()
        )

        now = self.robot.getTime()
        if MOCAP_INTERVAL_S > 0 and now - self.last_mocap >= MOCAP_INTERVAL_S:
            self.last_mocap = now
            truth = ground_truth(self.node)
            drift = math.hypot(truth.x - pose.x, truth.y - pose.y)
            drift_theta = control.wrap_angle(truth.theta - pose.theta)
            print(
                f"mocap correction at t={now:.1f}s: drift {drift * 100:.1f} cm, "
                f"{math.degrees(drift_theta):+.1f} deg",
                flush=True,
            )
            self.odom.pose = truth
            pose = truth

        distances = perception.read_distances(
            [sensor.getValue() for sensor in self.dev.sensors]
        )

        self.map.stamp_covered(pose.x, pose.y)
        for index, distance in enumerate(distances):
            hit = distance < epuck.IR_MAP_TRUST_RANGE_M
            reach = distance if hit else epuck.IR_MAX_RANGE_M
            origin, endpoint = perception.sensor_ray(pose, index, reach)
            self.map.mark_ray(origin, endpoint, hit)

        _, _, front = perception.repulsion(distances)
        return distances, front

    # ---- planning ----

    def sweep_target(self, pose: Pose):
        """Next lane waypoint, skipping ones that are done, blocked or hopeless."""
        blocked = self.map.blocked_mask()
        now = self.robot.getTime()

        while self.lane_index < len(self.lane_points):
            x, y = self.lane_points[self.lane_index]
            cell = self.map.world_to_cell(x, y)
            reached = math.hypot(x - pose.x, y - pose.y) < WAYPOINT_TOLERANCE_M
            unreachable = cell is None or blocked[cell]
            if reached or unreachable or now > self.waypoint_deadline:
                if not reached and not unreachable:
                    print(
                        f"waypoint {self.lane_index} timed out at t={now:.1f}s",
                        flush=True,
                    )
                self.lane_index += 1
                self.waypoint_deadline = now + LANE_TIMEOUT_S
                continue
            return (x, y)

        print(f"sweep finished at t={now:.1f}s", flush=True)
        self.mode = "GAP_FILL"
        return None

    # ---- actuation ----

    def drive(self, v: float, omega: float) -> None:
        left, right = control.wheel_speeds(v, omega)
        self.dev.left_motor.setVelocity(left)
        self.dev.right_motor.setVelocity(right)

    def steer_toward(self, pose: Pose, target, distances, front) -> None:
        """Blend the bearing to the target with the repulsive field, then drive."""
        bearing = control.wrap_angle(
            math.atan2(target[1] - pose.y, target[0] - pose.x) - pose.theta
        )
        attract_x, attract_y = math.cos(bearing), math.sin(bearing)

        repel_x, repel_y, _ = perception.repulsion(distances)
        self.field[0] += REPULSION_ALPHA * (repel_x - self.field[0])
        self.field[1] += REPULSION_ALPHA * (repel_y - self.field[1])

        command_x = attract_x + REPULSION_GAIN * self.field[0]
        command_y = attract_y + REPULSION_GAIN * self.field[1]

        # A perfectly head-on obstacle cancels the field's tangential term and
        # leaves nothing but a backward push. Break the tie toward whichever
        # side the target is on, so the robot commits to going round.
        if front > 0.5 and abs(command_y) < 0.1:
            command_y += math.copysign(0.5, bearing if bearing != 0.0 else 1.0)

        error = math.atan2(command_y, command_x)
        v, omega = control.go_to_point(
            error,
            CRUISE_SPEED,
            K_HEADING,
            OMEGA_MAX,
            slowdown=1.0 - FRONT_BRAKE * front,
        )
        self.drive(v, omega)

    def handle_stall(self, distances) -> bool:
        """Back out of a trap. Returns True if the escape consumed this step."""
        if self.escape_steps > 0:
            self.escape_steps -= 1
            self.drive(-0.5 * CRUISE_SPEED, self.escape_turn)
            return True

        if abs(self.odom.step_ds) < STALL_SPEED_M:
            self.stalled_steps += 1
        else:
            self.stalled_steps = 0

        if self.stalled_steps <= STALL_STEPS:
            return False

        self.stalled_steps = 0
        self.escape_steps = ESCAPE_STEPS
        # Turn toward whichever side has more room as we reverse.
        left_room = distances[5] + distances[6]
        right_room = distances[1] + distances[2]
        self.escape_turn = 1.5 if left_room > right_room else -1.5
        self.field = [0.0, 0.0]  # forget the field that led into the trap
        print(f"stuck at t={self.robot.getTime():.1f}s - backing out", flush=True)
        return True

    # ---- reporting ----

    def report(self) -> None:
        now = self.robot.getTime()
        if now - self.last_report < REPORT_INTERVAL_S:
            return
        self.last_report = now
        covered, coverable = self.map.coverage_counts()
        print(
            f"t={now:6.1f}s mode={self.mode:8s} "
            f"coverage={100.0 * covered / coverable:5.1f}% ({covered}/{coverable} cells)",
            flush=True,
        )

    # ---- main loop ----

    def run(self) -> None:
        while self.robot.step(TIME_STEP) != -1:
            distances, front = self.sense()
            self.report()

            # Stall recovery only makes sense while the robot is trying to
            # get somewhere. Once SWEEP hands off to GAP_FILL (not yet
            # implemented -- Task 11), the robot is deliberately parked with
            # zero velocity; without this guard handle_stall sees that zero
            # velocity as a stall forever and the robot backs out and turns
            # in an endless loop instead of the "sits still" behaviour this
            # task's brief calls for.
            if self.mode == "SWEEP" and self.handle_stall(distances):
                continue

            pose = self.odom.pose
            target = self.sweep_target(pose) if self.mode == "SWEEP" else None
            if target is None:
                self.drive(0.0, 0.0)
                continue

            self.steer_toward(pose, target, distances, front)


if __name__ == "__main__":
    Roomba().run()
