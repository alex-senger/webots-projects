"""Draw the 3-DoF planar arm for one or more joint configurations."""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fk import LINKS_MM, forward_kinematics, joint_positions

CONFIGURATIONS = [
    (25.0, 50.0, -35.0),
    (100.0, -40.0, 25.0),
    (175.0, 35.0, 40.0),
    (-50.0, -25.0, -15.0),
]

OUTPUT: Path = Path(__file__).resolve().parents[1] / "docs" / "figures" / "fk_arm.png"


def main() -> None:
    reach = sum(LINKS_MM)
    fig, ax = plt.subplots(figsize=(6, 6))

    for angles in CONFIGURATIONS:
        points = joint_positions(*angles)
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        x, y, phi = forward_kinematics(*angles)
        label = (
            f"({angles[0]:g}, {angles[1]:g}, {angles[2]:g})deg "
            f"-> ({x:.1f}, {y:.1f}) mm, phi={phi:.0f}deg"
        )
        ax.plot(xs, ys, marker="o", linewidth=2, label=label)

    ax.add_artist(plt.Circle((0, 0), reach, fill=False, linestyle=":", alpha=0.4))
    ax.set_xlim(-reach * 1.1, reach * 1.1)
    ax.set_ylim(-reach * 1.1, reach * 1.1)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title("3-DoF planar arm: a1=30, a2=25, a3=20 mm")
    ax.legend(fontsize=7, loc="lower left")

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUTPUT, dpi=200, bbox_inches="tight")
    print(f"wrote {OUTPUT}")


if __name__ == "__main__":
    main()
