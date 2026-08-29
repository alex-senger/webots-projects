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

import numpy as np
from controller import Supervisor

from epucklib import control, devices, epuck, grid, perception, sweep, trace
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
# Tighter spacings (0.04/0.045) lost more to lane-transition corner-cutting
# than they gained in overlap; 0.06 left visible gaps. 0.05 was the empirical
# optimum over 13 tuning runs, and the relationship is non-monotonic.
LANE_SPACING_M = 0.05  # tighter than the 7.4 cm body, so lanes overlap

# ---- motion ----
CRUISE_SPEED = 0.09  # m/s, about 70% of what the motors can do
K_HEADING = 4.0
OMEGA_MAX = 4.0
# 0.03 cut corners at lane ends; 0.01 caused waypoint timeouts. Tuned
# empirically alongside LANE_SPACING_M over the same 13 runs.
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
TIME_BUDGET_S = 600.0  # hard stop, so a batch run always terminates
PROGRESS_WINDOW_S = 60.0  # how long coverage may stagnate before giving up
PROGRESS_MIN = 0.005  # 0.5 percentage points counts as progress

TRACE_FIELDS = [
    "t_s", "mode", "x", "y", "theta",
    "gt_x", "gt_y", "drift_m", "coverage",
]


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
        self.stall = control.StallDetector(STALL_SPEED_M, STALL_STEPS, ESCAPE_STEPS)
        self.mode_before_escape = self.mode

        self.last_mocap = self.robot.getTime()
        self.last_report = 0.0

        self.path: list[tuple[float, float]] = []  # pending gap-fill waypoints
        self.trace = trace.CsvTrace(
            trace.trace_path(CONTROLLER_NAME, REPO_ROOT), TRACE_FIELDS
        )
        self.best_coverage = 0.0
        self.best_coverage_time = 0.0

    # ---- perception ----

    def sense(self) -> tuple[list[float], tuple[float, float, float]]:
        """Update the pose and the map.

        Returns the ring of IR distances and the repulsion triple
        (field x, field y, front proximity) derived from it. The triple is
        computed once here and handed to the steering, rather than being
        recomputed there from the same distances.
        """
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
            # The ray only ever reaches as far as the sensor actually saw:
            # `raw_to_distance` already saturates at IR_MAX_RANGE_M when there
            # is nothing in range, so extending a genuine 6 cm reading to 7 cm
            # would clear cells that demonstrably hold something.
            hit = distance < epuck.IR_MAP_TRUST_RANGE_M
            origin, endpoint = perception.sensor_ray(pose, index, distance)
            self.map.mark_ray(origin, endpoint, hit)

        return distances, perception.repulsion(distances)

    # ---- planning ----

    def sweep_target(self, pose: Pose) -> tuple[float, float] | None:
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

    def gap_fill_target(self, pose: Pose) -> tuple[float, float] | None:
        """Next waypoint on the way to the closest cell still worth sweeping.

        The queued path is thrown away, and a fresh one planned, whenever it
        runs out *or* its next waypoint has just become blocked -- a box
        discovered since the plan was made inflates over the cells around it,
        and steering at a waypoint inside that inflation would drive the robot
        straight at the thing it just found. Checking only the head of the
        queue is enough: every later waypoint gets the same test on the step
        it becomes the head.
        """
        blocked = self.map.blocked_mask()
        if self.path:
            cell = self.map.world_to_cell(*self.path[0])
            if cell is None or blocked[cell]:
                self.path.clear()

        while self.path:
            x, y = self.path[0]
            if math.hypot(x - pose.x, y - pose.y) < WAYPOINT_TOLERANCE_M:
                self.path.pop(0)
                continue
            return (x, y)

        start = self.map.world_to_cell(pose.x, pose.y)
        if start is None:
            self.finish("robot left the arena")
            return None

        cells = self.map.nearest_uncovered(start)
        if cells is None:
            self.finish("nothing reachable left to cover")
            return None

        # The first cell is where the robot already stands, so drop it.
        self.path = [self.map.cell_center(*cell) for cell in self.map.shortcut(cells)[1:]]
        if not self.path:
            self.finish("nothing reachable left to cover")
            return None
        return self.path[0]

    # ---- actuation ----

    def drive(self, v: float, omega: float) -> None:
        left, right = control.wheel_speeds(v, omega)
        self.dev.left_motor.setVelocity(left)
        self.dev.right_motor.setVelocity(right)

    def steer_toward(
        self,
        pose: Pose,
        target: tuple[float, float],
        repulsion: tuple[float, float, float],
    ) -> None:
        """Blend the bearing to the target with the repulsive field, then drive."""
        repel_x, repel_y, front = repulsion

        bearing = control.wrap_angle(
            math.atan2(target[1] - pose.y, target[0] - pose.x) - pose.theta
        )

        # Exponential smoothing lives here rather than in the library: it is
        # per-step state belonging to this control loop, and the escape
        # manoeuvre resets it.
        self.field[0] += REPULSION_ALPHA * (repel_x - self.field[0])
        self.field[1] += REPULSION_ALPHA * (repel_y - self.field[1])

        error = control.blend_command(
            bearing, self.field[0], self.field[1], front, REPULSION_GAIN
        )
        v, omega = control.go_to_point(
            error,
            CRUISE_SPEED,
            K_HEADING,
            OMEGA_MAX,
            slowdown=1.0 - FRONT_BRAKE * front,
        )
        self.drive(v, omega)

    def handle_stall(self, distances: list[float]) -> bool:
        """Back out of a trap. Returns True if the escape consumed this step."""
        turn = self.stall.update(self.odom.step_ds, distances)
        if turn is None:
            return False

        if self.stall.fired:
            # Flag the manoeuvre in the trace so a backing-out wiggle is
            # distinguishable from ordinary driving. DONE is terminal and must
            # never be overwritten -- run() short-circuits before we get here,
            # but the guard keeps that a local property.
            if self.mode != "DONE":
                self.mode_before_escape = self.mode
                self.mode = "ESCAPE"
            self.field = [0.0, 0.0]  # forget the field that led into the trap
            print(f"stuck at t={self.robot.getTime():.1f}s - backing out", flush=True)
            return True

        self.drive(-0.5 * CRUISE_SPEED, turn)
        if not self.stall.escaping and self.mode == "ESCAPE":
            self.mode = self.mode_before_escape
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

    def check_progress(self) -> None:
        """Stop if coverage has stalled or the time budget has run out."""
        now = self.robot.getTime()
        fraction = self.map.coverage_fraction()
        if fraction > self.best_coverage + PROGRESS_MIN:
            self.best_coverage = fraction
            self.best_coverage_time = now
        elif now - self.best_coverage_time > PROGRESS_WINDOW_S:
            self.finish(f"no progress for {PROGRESS_WINDOW_S:.0f}s")
        if now > TIME_BUDGET_S:
            self.finish(f"time budget of {TIME_BUDGET_S:.0f}s reached")

    def finish(self, reason: str) -> None:
        """Stop the robot, write the map, print the summary, and end the run."""
        if self.mode == "DONE":
            return
        self.mode = "DONE"
        self.drive(0.0, 0.0)

        covered, coverable = self.map.coverage_counts()
        area = covered * self.map.cell * self.map.cell
        print(
            f"\n=== coverage complete: {reason} ===\n"
            f"    time      {self.robot.getTime():.1f} s\n"
            f"    coverage  {100.0 * covered / coverable:.1f}% "
            f"({covered}/{coverable} cells, {area:.3f} m^2)\n"
            f"    obstacles {(self.map.state == grid.OCCUPIED).sum()} cells discovered",
            flush=True,
        )

        np.savez_compressed(
            REPO_ROOT / "analysis" / "traces" / f"{CONTROLLER_NAME}_map.npz",
            covered=self.map.covered,
            state=self.map.state,
            cell_size_m=self.map.cell,
            half_extent_m=self.map.half,
        )
        self.trace.close()

        # Tear the simulation down, so `webots --batch --mode=fast` returns
        # instead of spinning forever on a robot that is already parked. The
        # quit only takes effect at the next step boundary, so the DONE guard
        # in run() still has to hold the motors at zero until then.
        self.robot.simulationQuit(0)

    def record(self, pose: Pose) -> None:
        truth = ground_truth(self.node)
        self.trace.write(
            t_s=f"{self.robot.getTime():.3f}",
            mode=self.mode,
            x=f"{pose.x:.4f}",
            y=f"{pose.y:.4f}",
            theta=f"{pose.theta:.4f}",
            gt_x=f"{truth.x:.4f}",
            gt_y=f"{truth.y:.4f}",
            drift_m=f"{math.hypot(truth.x - pose.x, truth.y - pose.y):.4f}",
            coverage=f"{self.map.coverage_fraction():.4f}",
        )

    # ---- main loop ----

    def run(self) -> None:
        while self.robot.step(TIME_STEP) != -1:
            distances, repulsion = self.sense()
            pose = self.odom.pose
            self.record(pose)
            self.report()

            if self.mode == "DONE":
                self.drive(0.0, 0.0)
                continue

            self.check_progress()
            if self.mode == "DONE":
                self.drive(0.0, 0.0)
                continue

            if self.handle_stall(distances):
                continue

            if self.mode == "SWEEP":
                target = self.sweep_target(pose)
            else:
                target = self.gap_fill_target(pose)

            if target is None:
                self.drive(0.0, 0.0)
                continue

            self.steer_toward(pose, target, repulsion)


if __name__ == "__main__":
    Roomba().run()
