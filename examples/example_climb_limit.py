"""Show how a bounded climb angle creates helical turns.

Run from the repository root: python -m examples.example_climb_limit
Use --plot for a labeled figure or --save-visuals for PNG and GIF files.
The aircraft must finish above the same (x, y) position and with the same yaw.
It cannot simply rise vertically: it keeps moving forward in circular turns.
"""

from math import degrees, radians
from pathlib import Path
import sys

from dubins_airplane import Pose, plan_dubins_airplane


def main():
    start = Pose(0, 0, 100, radians(0))
    radius = 50  # metres
    max_angle = radians(12)

    print("Target z (m) | Full turns | 3D length (m) | Flight-path angle")
    for target_z in (100, 150, 300, 500):
        goal = Pose(0, 0, target_z, start.yaw)
        path = plan_dubins_airplane(start, goal, radius, max_angle)
        print(f"{target_z:12.0f} | {path.extra_turns:10d} | "
              f"{path.length:13.1f} | {degrees(path.flight_path_angle):6.1f} deg")

    # Inspect one helix at equal fractions of its horizontal travel.
    path = plan_dubins_airplane(start, Pose(0, 0, 300, start.yaw),
                               radius, max_angle)
    print("\n300 m target: points along the helix")
    for fraction in (0, 0.25, 0.5, 0.75, 1):
        pose = path.pose_at(fraction * path.horizontal_length)
        print(f"  {fraction:>4.0%}: x={pose.x:7.1f}, y={pose.y:7.1f}, "
              f"z={pose.z:7.1f} m, yaw={degrees(pose.yaw):6.1f} deg")

    if "--plot" in sys.argv or "--save-visuals" in sys.argv:
        from dubins_airplane.visualization import RouteLeg, Waypoint, visualize_routes

        save = "--save-visuals" in sys.argv
        visuals = Path(__file__).resolve().parents[1] / "visuals"
        visualize_routes(
            [RouteLeg("Three-turn climb", path)],
            [Waypoint("Start", start), Waypoint("Target above start", path.goal)],
            "Climb above the same point without flying vertically",
            png=visuals / "climb_helix.png" if save else None,
            gif=visuals / "climb_helix.gif" if save else None,
            show="--plot" in sys.argv,
        )


if __name__ == "__main__":
    main()
