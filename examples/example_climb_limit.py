"""Compare Owen's low, medium, and high altitude-gain constructions.

Run from the repository root: python -m examples.example_climb_limit
Use --plot for a labeled figure or --save-visuals for PNG and GIF files.
Each target has the same (x, y, yaw), but a different altitude. The distance
between start and goal is comfortably above the reference's 6*R_min bound.
"""

from math import degrees, radians, tan, tau
from pathlib import Path
import sys

from dubins_airplane import Pose, plan_dubins_airplane


def main():
    start = Pose(0, 0, 100, radians(0))
    radius = 50  # metres
    max_angle = radians(12)
    level_goal = Pose(500, 200, start.z, radians(90))
    base = plan_dubins_airplane(start, level_goal, radius, max_angle).horizontal_length
    alternatives = []

    print("Case   | Word | Target z | Radius | Turns | 3D length | Climb angle")
    for label, required in (("low", 0.5 * base),
                            ("medium", base + 0.5 * tau * radius),
                            ("high", base + 2.5 * tau * radius)):
        goal = Pose(level_goal.x, level_goal.y, start.z + required * tan(max_angle),
                    level_goal.yaw)
        path = plan_dubins_airplane(start, goal, radius, max_angle)
        alternatives.append((label, path))
        print(f"{path.altitude_case:6} | {path.word:4} | {goal.z:8.1f} | "
              f"{path.radius:6.1f} | {path.extra_turns:5d} | "
              f"{path.length:9.1f} | {degrees(path.flight_path_angle):5.1f} deg")

    # The middle target uses a fitted partial maneuver, not a complete loop.
    path = alternatives[1][1]
    print("\nMedium case: points at equal horizontal-distance fractions")
    for fraction in (0, 0.25, 0.5, 0.75, 1):
        pose = path.pose_at(fraction * path.horizontal_length)
        print(f"  {fraction:>4.0%}: x={pose.x:7.1f}, y={pose.y:7.1f}, "
              f"z={pose.z:7.1f} m, yaw={degrees(pose.yaw):6.1f} deg")

    if "--plot" in sys.argv or "--save-visuals" in sys.argv:
        from dubins_airplane.visualization import RouteLeg, Waypoint, visualize_routes

        save = "--save-visuals" in sys.argv
        visuals = Path(__file__).resolve().parents[1] / "visuals"
        visualize_routes(
            [RouteLeg(label.capitalize(), candidate) for label, candidate in alternatives],
            [Waypoint("Start", start),
             *[Waypoint(f"{label.capitalize()} target", candidate.goal)
               for label, candidate in alternatives]],
            "Owen: low, medium and high altitude gain",
            png=visuals / "climb_helix.png" if save else None,
            gif=visuals / "climb_helix.gif" if save else None,
            show="--plot" in sys.argv,
            compare_alternatives=True,
        )


if __name__ == "__main__":
    main()
