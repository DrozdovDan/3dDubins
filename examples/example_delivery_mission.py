"""Connect several delivery-area fly-through poses and export a 3D route.

Run from the repository root: python -m examples.example_delivery_mission
Use --plot for a labeled figure or --save-visuals for PNG and GIF files.

These are route waypoints, not a landing, payload-release, or safety plan.
"""

import csv
from math import degrees, radians
from pathlib import Path
import sys

from dubins_airplane import Pose, plan_dubins_airplane


def main():
    # East, north, altitude (metres), and travel heading (radians from east).
    waypoints = [
        ("Depot", Pose(0, 0, 120, radians(0))),
        ("Customer A", Pose(500, 200, 250, radians(90))),
        ("Customer B", Pose(900, 600, 180, radians(180))),
        ("Return corridor", Pose(0, 0, 120, radians(180))),
    ]
    radius = 60  # minimum horizontal turn radius, metres
    max_angle = radians(15)
    legs = []
    for (start_name, start), (goal_name, goal) in zip(waypoints, waypoints[1:]):
        path = plan_dubins_airplane(start, goal, radius, max_angle)
        legs.append((start_name, goal_name, path))

    project_root = Path(__file__).resolve().parents[1]
    output = project_root / "outputs" / "delivery_mission_route.csv"
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["leg", "from", "to", "x_m", "y_m", "altitude_m", "yaw_deg"])
        for number, (start_name, goal_name, path) in enumerate(legs, 1):
            for pose in path.sample(max_horizontal_step=10):
                writer.writerow([number, start_name, goal_name, pose.x, pose.y,
                                 pose.z, degrees(pose.yaw)])
            print(f"{number}. {start_name} -> {goal_name}: {path.word}, "
                  f"{path.extra_turns} helix turns, {path.length:.1f} m")

    print(f"Total flight-path distance: {sum(path.length for _, _, path in legs):.1f} m")
    print(f"Wrote {output}")

    if "--plot" in sys.argv or "--save-visuals" in sys.argv:
        from dubins_airplane.visualization import RouteLeg, Waypoint, visualize_routes

        save = "--save-visuals" in sys.argv
        visuals = project_root / "visuals"
        visualize_routes(
            [RouteLeg(f"{start_name} to {goal_name}", path)
             for start_name, goal_name, path in legs],
            [Waypoint(name, pose) for name, pose in waypoints],
            "Three-leg aerial delivery-area route",
            png=visuals / "delivery_mission.png" if save else None,
            gif=visuals / "delivery_mission.gif" if save else None,
            show="--plot" in sys.argv,
        )


if __name__ == "__main__":
    main()
