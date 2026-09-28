"""Run from the repository root with ``python -m examples.example``.

It plans two aircraft route legs and writes sampled 3D poses to outputs/.
Use --plot for a labeled figure or --save-visuals for PNG and GIF files.
"""

import csv
from math import degrees, radians
from pathlib import Path
import sys

from dubins_airplane import Pose, plan_dubins_airplane


def main():
    # East, north, altitude (metres), heading (radians from east).
    depot = Pose(0.0, 0.0, 120.0, radians(0))
    # Both altitude changes exercise Owen's high case: turns plus a fitted radius.
    transfer_point = Pose(500.0, 250.0, 500.0, radians(90))
    destination = Pose(850.0, 600.0, 100.0, radians(180))

    minimum_turn_radius = 60.0  # m, horizontal projection
    maximum_climb_angle = radians(15)
    legs = [
        plan_dubins_airplane(depot, transfer_point,
                             minimum_turn_radius, maximum_climb_angle),
        plan_dubins_airplane(transfer_point, destination,
                             minimum_turn_radius, maximum_climb_angle),
    ]

    project_root = Path(__file__).resolve().parents[1]
    output = project_root / "outputs" / "example_route.csv"
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["leg", "x_m", "y_m", "altitude_m", "yaw_deg"])
        for number, leg in enumerate(legs, 1):
            for pose in leg.sample(max_horizontal_step=5.0):
                writer.writerow([number, pose.x, pose.y, pose.z, degrees(pose.yaw)])
            print(f"Leg {number}: {leg.altitude_case}, {leg.word}, "
                  f"R={leg.radius:.1f} m, {leg.extra_turns} full helix turns, "
                  f"{leg.length:.1f} m, flight-path angle "
                  f"{degrees(leg.flight_path_angle):.1f} deg")
    print(f"Wrote {output}")

    if "--plot" in sys.argv or "--save-visuals" in sys.argv:
        from dubins_airplane.visualization import RouteLeg, Waypoint, visualize_routes

        save = "--save-visuals" in sys.argv
        visuals = project_root / "visuals"
        visualize_routes(
            [RouteLeg("Depot to transfer", legs[0]),
             RouteLeg("Transfer to destination", legs[1])],
            [Waypoint("Depot", depot), Waypoint("Transfer", transfer_point),
             Waypoint("Destination", destination)],
            "Two-leg Dubins-airplane route with climb and descent",
            png=visuals / "two_leg_route.png" if save else None,
            gif=visuals / "two_leg_route.gif" if save else None,
            show="--plot" in sys.argv,
        )


if __name__ == "__main__":
    main()
