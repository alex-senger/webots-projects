"""Interactive forward kinematics for a 3-DoF planar manipulator."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fk import LINKS_MM, forward_kinematics


def ask_angle(prompt: str) -> float:
    """Read one angle in degrees, re-prompting until it parses."""
    while True:
        raw = input(prompt).strip()
        try:
            return float(raw)
        except ValueError:
            print(f"  '{raw}' is not a number — please enter an angle in degrees.")


def main() -> None:
    a1, a2, a3 = LINKS_MM
    print("3-DoF planar manipulator — forward kinematics")
    print(f"Link lengths: a1 = {a1:g} mm, a2 = {a2:g} mm, a3 = {a3:g} mm")
    print("Enter the three joint angles in degrees.\n")

    t1 = ask_angle("  theta1 (deg): ")
    t2 = ask_angle("  theta2 (deg): ")
    t3 = ask_angle("  theta3 (deg): ")

    x, y, phi = forward_kinematics(t1, t2, t3)

    print("\nEnd-effector pose:")
    print(f"  x   = {x:8.3f} mm")
    print(f"  y   = {y:8.3f} mm")
    print(f"  phi = {phi:8.3f} deg")


if __name__ == "__main__":
    main()
