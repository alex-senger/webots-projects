"""Render the coverage run as a figure.

Left: what the robot ended up believing about the floor -- discovered
obstacles, swept ground, ground it could never sweep, and the path it
actually drove. Right: how coverage accumulated over time, with the handover
from the lane sweep to gap filling marked.

Run after a simulation:  uv run python analysis/plot_coverage.py
"""

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from epucklib.epuck import BODY_RADIUS_M  # noqa: E402
from epucklib.grid import OCCUPIED, CoverageMap  # noqa: E402

TRACES = REPO_ROOT / "analysis" / "traces"
CONTROLLER_NAME = "epuck_roomba"
OUTPUT = REPO_ROOT / "docs" / "figures" / "roomba_coverage.png"


def load_trace(path):
    times, xs, ys, coverage, modes = [], [], [], [], []
    with path.open() as handle:
        for row in csv.DictReader(handle):
            times.append(float(row["t_s"]))
            xs.append(float(row["gt_x"]))
            ys.append(float(row["gt_y"]))
            coverage.append(float(row["coverage"]) * 100.0)
            modes.append(row["mode"])
    return times, xs, ys, coverage, modes


def main():
    map_path = TRACES / f"{CONTROLLER_NAME}_map.npz"
    trace_path = TRACES / f"{CONTROLLER_NAME}.csv"
    for path in (map_path, trace_path):
        if not path.exists():
            raise SystemExit(f"{path} is missing -- run the simulation first")

    data = np.load(map_path)
    covered, state = data["covered"], data["state"]
    half = float(data["half_extent_m"])
    cell_size = float(data["cell_size_m"])
    extent = (-half, half, -half, half)
    n = state.shape[0]

    times, xs, ys, coverage, modes = load_trace(trace_path)

    # Ask the grid itself which cells the footprint could reach; a fork of the
    # rule here would go stale the moment the geometry changed.
    geometry = CoverageMap(
        half_extent_m=half, cell_size_m=cell_size, robot_radius_m=BODY_RADIUS_M
    )
    stampable = geometry.stampable_mask()
    out_of_reach = ~stampable & (state != OCCUPIED)

    # 0 = untouched floor, 1 = swept, 2 = obstacle, 3 = structurally out of reach.
    picture = np.where(covered, 1, 0)
    picture[out_of_reach] = 3
    picture[state == OCCUPIED] = 2
    palette = ListedColormap(["#f2efe6", "#4c9f70", "#37383a", "#d9d9d6"])

    coverable_all = state != OCCUPIED
    overall_pct = 100.0 * (covered & coverable_all).sum() / coverable_all.sum()
    reachable_mask = coverable_all & stampable
    reachable_pct = 100.0 * (covered & reachable_mask).sum() / reachable_mask.sum()

    figure, (left, right) = plt.subplots(1, 2, figsize=(12, 5.2))

    left.imshow(picture, origin="lower", extent=extent, cmap=palette, vmin=0, vmax=3)
    (path_line,) = left.plot(xs, ys, color="#c1440e", linewidth=0.8, alpha=0.9)
    (start_dot,) = left.plot(xs[0], ys[0], "o", color="#c1440e", markersize=6)
    left.set_title(
        f"Coverage {overall_pct:.1f}% overall\n"
        f"({reachable_pct:.1f}% of reachable floor)"
    )
    left.set_xlabel("x (m)")
    left.set_ylabel("y (m)")
    left.set_aspect("equal")

    path_line.set_label("path driven")
    start_dot.set_label("start")
    swatches = [
        plt.Rectangle((0, 0), 1, 1, color="#4c9f70", label="swept"),
        plt.Rectangle((0, 0), 1, 1, color="#f2efe6", ec="#999", label="missed"),
        plt.Rectangle((0, 0), 1, 1, color="#37383a", label="obstacle found"),
        plt.Rectangle((0, 0), 1, 1, color="#d9d9d6", ec="#999", label="out of reach"),
    ]
    left.legend(
        handles=[*swatches, path_line, start_dot],
        loc="upper left",
        bbox_to_anchor=(1.02, 1.0),
        fontsize=8,
    )

    right.plot(times, coverage, color="#4c9f70", linewidth=2)
    right.set_title(f"Coverage over time (final {coverage[-1]:.1f}%)")
    right.set_xlabel("simulated time (s)")
    right.set_ylabel("coverage (%)")
    right.set_ylim(0, 100)
    right.grid(alpha=0.3)

    # Mark where the lane sweep handed over to gap filling.
    for index, mode in enumerate(modes):
        if mode == "GAP_FILL":
            right.axvline(times[index], color="#c1440e", linestyle="--", linewidth=1)
            right.text(times[index], 5, " sweep done", color="#c1440e", fontsize=8)
            break

    figure.tight_layout()
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(OUTPUT, dpi=150, bbox_inches="tight")
    print(f"wrote {OUTPUT.relative_to(REPO_ROOT)}")
    print(f"overall coverage: {overall_pct:.1f}%  reachable-only: {reachable_pct:.1f}%")
    print(f"out-of-reach cells: {int(out_of_reach.sum())} / {n * n}")


if __name__ == "__main__":
    main()
