"""Compare two level-flight approaches to the same delivery location.

Run from the repository root: python -m examples.example_arrival_heading
Use --plot for a labeled figure or --save-visuals for PNG and GIF files.
Only the required arrival heading changes; the endpoint position is identical.
"""

from math import degrees, radians
from pathlib import Path
import sys

from dubins_airplane import Pose, plan_dubins_airplane


def main():
    depot = Pose(0, 0, 120, radians(0))
    delivery_xy = (450, 250)
    radius = 60  # minimum horizontal turn radius, metres
    max_angle = radians(15)
    alternatives = []

    for label, heading in (("east-facing arrival", 0),
                           ("west-facing arrival", 180)):
        goal = Pose(*delivery_xy, 120, radians(heading))
        path = plan_dubins_airplane(depot, goal, radius, max_angle)
        alternatives.append((label, path))
        print(f"{label}: {path.word}, {path.length:.1f} m")
        print("  segments:", ", ".join(
            f"{segment.kind} {segment.horizontal_length:.1f} m"
            for segment in path.segments
        ))
        end = path.pose_at(path.horizontal_length)
        print(f"  endpoint: x={end.x:.1f}, y={end.y:.1f}, "
              f"z={end.z:.1f}, yaw={degrees(end.yaw):.0f} deg")

    if "--plot" in sys.argv or "--save-visuals" in sys.argv:
        from dubins_airplane.visualization import RouteLeg, Waypoint, visualize_routes

        save = "--save-visuals" in sys.argv
        visuals = Path(__file__).resolve().parents[1] / "visuals"
        visualize_routes(
            [RouteLeg(label, path) for label, path in alternatives],
            [Waypoint("Depot", depot),
             Waypoint("Delivery point", alternatives[0][1].goal)],
            "Same destination, different arrival headings",
            png=visuals / "arrival_headings.png" if save else None,
            gif=visuals / "arrival_headings.gif" if save else None,
            show="--plot" in sys.argv,
            compare_alternatives=True,
        )


if __name__ == "__main__":
    main()
