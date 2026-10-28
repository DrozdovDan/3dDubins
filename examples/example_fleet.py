"""Airborne fleet examples: python -m examples.example_fleet --scenario hills.

For a shared-clock 3D animation: python -m rerun_scripts.fleet --scenario hills.
All circles are separate, all agents exist from t=0, and no intermediate
route poses are hand-picked. CLI requires C++ by default.
"""

import argparse
import csv
import json
from math import ceil, exp, hypot, pi, radians, sin
import os
from pathlib import Path

from dubins_airplane import (AgentRequest, BoxObstacle, ElevationMap, FlightWorld,
                            DecentralizedPlanningError, NetworkSettings, backend_name,
                            plan_fleet, simulate_decentralized)
from dubins_airplane.fleet import check_separation, sample_flight

SCENARIOS = {
    "hills": "Три самолёта: холмы и подъём на плато",
    "mountain_pass": "Три самолёта: горный проход и два барьера",
    "crossing": "Два самолёта: пересечение диагональных маршрутов",
    "three_way": "Три самолёта: общий район пересечения",
    "bottleneck": "Три самолёта: встречные маршруты через два барьера",
    "altitude_conflict_1km": "Встречные маршруты: исходный конфликт и перепад высот 1000 м",
    "mountain_exchange_5": "Пять самолётов: горный транзит, три барьера и перепад 1000 м",
}

SCENARIO_PLANNING_WINDOWS = {"mountain_exchange_5": 15.0}


def make_fleet_scenario(name="hills"):
    if name not in SCENARIOS:
        raise ValueError(f"unknown fleet scenario: {name}")
    if name == "mountain_exchange_5":
        def height(x, y):
            s = max(0, min(1, (x - 250) / 5500))
            mountains = (300 * exp(-((x-2400)/550)**2 - ((y-550)/380)**2)
                         + 270 * exp(-((x-3300)/600)**2 - ((y+600)/400)**2)
                         + 180 * exp(-((x-4300)/500)**2 - ((y-450)/350)**2)
                         - 100 * exp(-((x-3000)/800)**2 - (y/240)**2))
            return max(0, min(1000, 1000*s + sin(pi*s)**2 * mountains))
        terrain = ElevationMap.from_function(-200, -1200, 10, 640, 240, height)
        boxes = (BoxObstacle(1800, -1200, 0, 1940, -60, 1600),
                 BoxObstacle(3460, 60, 0, 3600, 1200, 1600),
                 BoxObstacle(4700, -1200, 0, 4840, -60, 1600))
        world = FlightWorld(terrain, 40, 260, obstacles=boxes, obstacle_margin=5)
        specifications = (("A", (0, -700), (6000, 700)),
                          ("B", (6000, -350), (0, -350)),
                          ("C", (0, 0), (6000, 0)),
                          ("D", (6000, 350), (0, 350)),
                          ("E", (0, 700), (6000, -700)))
        requests = tuple(AgentRequest(n, world.choose_loiter(*a, 40), world.choose_loiter(*b, 40),
                                     speed=14.6 if n == "D" else 20, radius=40, max_angle=radians(15),
                                     earliest_departure=25,
                                     initial_phase=-(14.6 if n == "D" else 20)*20/40)
                         for n, a, b in specifications)
        # Back-rotate each physical circle phase and shift all original flights
        # by the same 20 s. This adds coordination slack WITHOUT resolving any
        # relative collision or changing the before/after departure comparison.
        return world, requests
    if name == "altitude_conflict_1km":
        # Flat endpoint terraces and a flyable 10.3-degree hillside between them.
        # Both terrain AND start-to-goal altitude change by exactly 1000 m.
        terrain = ElevationMap.from_function(-200, -800, 10, 640, 160,
                                             lambda x, y: 1000 * max(0, min(1, (x - 250) / 5500)))
        world = FlightWorld(terrain, 40, 200)
        requests = tuple(AgentRequest(n, world.choose_loiter(*a, 40), world.choose_loiter(*b, 40),
                                     speed=20, radius=40, max_angle=radians(15))
                         for n, a, b in (("A", (0, -300), (6000, 300)),
                                         ("B", (6000, -300), (0, 300))))
        return world, requests
    if name == "mountain_pass":
        # Reuse the actual single-airplane terrain, including both peaks,
        # the plateau and forbidden volumes, not a decorative height mesh.
        from .example_obstacles import make_scenario
        world = make_scenario("mountain_pass").world
    else:
        height = (lambda x, y: (
            80 / (1 + exp(-(x - 530) / 100))
            + 95 * exp(-((x - 370) / 130)**2 - ((y - 180) / 150)**2)
            + 65 * exp(-((x - 540) / 130)**2 - ((y + 170) / 150)**2)
        )) if name == "hills" else (lambda x, y: 0)
        terrain = ElevationMap.from_function(-180, -400, 10, 118, 80, height)
        boxes = (BoxObstacle(310, -400, 0, 350, 80, 260),
                 BoxObstacle(510, -80, 0, 550, 400, 260)) if name == "bottleneck" else ()
        world = FlightWorld(terrain, 40, 110, obstacles=boxes, obstacle_margin=5)
    specifications = [("A", (0, -250), (800, 250), 20),
                      ("B", (0, 250), (800, -250), 20)]
    if name == "three_way":
        specifications.append(("C", (400, -280), (400, 280), 12))
    elif name in ("bottleneck", "hills", "mountain_pass"):
        specifications = [("A", (0, -220), (800, 220), 20),
                          ("B", (800, -220), (0, 220), 20),
                          ("C", (0, 40), (800, 40), 20)]
    requests = tuple(AgentRequest(n, world.choose_loiter(*a, 40), world.choose_loiter(*b, 40),
                                  speed=speed, radius=40, max_angle=radians(15))
                     for n, a, b, speed in specifications)
    return world, requests


def print_schedule(plan):
    print(f"Routing: {plan.routing_ms:.2f} ms; scheduling: {plan.scheduling_ms:.2f} ms")
    print(f"Separation > {plan.separation:g} m; all-airborne horizon {plan.horizon:.2f} s")
    print("Agent   speed    departure    arrival    wait laps    route length")
    for f in plan.flights:
        if f.duration == 0:
            print(f"{f.request.name:5} {f.request.speed:7.1f}   HOLD: keeps the initial circle indefinitely")
            continue
        print(f"{f.request.name:5} {f.request.speed:7.1f} {f.departure:12.2f} {f.arrival:10.2f}"
              f" {f.waiting_laps:12} {f.route.length:13.1f}")


def closest_approach(a, b, end, step=.25):
    """Diagnostic sampled minimum + local refinement, NOT a safety certificate."""
    def distance(at):
        p, q = a.pose_at(at), b.pose_at(at)
        return hypot(p.x - q.x, p.y - q.y, p.z - q.z)
    count = max(1, ceil(end / step))
    best = min((distance(end * k / count), end * k / count) for k in range(count + 1))
    left, right = max(0, best[1] - step), min(end, best[1] + step)
    for _ in range(40):
        one, two = (2 * left + right) / 3, (left + 2 * right) / 3
        if distance(one) < distance(two):
            right = two
        else:
            left = one
    at = (left + right) / 2
    minimum, at = min(best, (distance(at), at))
    return {"distance_m": minimum, "time_s": at, "method": "sampled + local refinement"}


def compare_initial_plan(result):
    """Observer-only comparison of exact original routes before peer negotiation.

    Initial flights retain physical t=0 phases and the earliest eligible circle
    departures, but deliberately ignore peers. Continuous checking certifies the
    conflict/safety decisions; numerical minima are only explanatory metrics.
    """
    initial, plan = result.initial_flights, result.plan
    if len(initial) != len(plan.flights):
        raise ValueError("cannot compare: some agents did not find an initial route")
    final = {f.request.name: f for f in plan.flights}
    end = min(plan.horizon, max(f.arrival + f.lap_period for f in (*initial, *plan.flights)))
    pairs = []
    for i, a in enumerate(initial):
        for b in initial[:i]:
            check = check_separation(a, b, plan.separation, plan.horizon)
            pairs.append({"agents": [b.request.name, a.request.name], "initial_safe": check.safe,
                          "initial_violation_s": None if check.safe else check.time,
                          "initial_closest": closest_approach(a, b, end),
                          "final_closest": closest_approach(final[a.request.name], final[b.request.name], end)})
    return {"description": "Independent routes before peer agreement, not flown in the coordinated mission",
            "pairs": pairs,
            "altitude_changes_m": {f.request.name: f.request.goal.z - f.request.start.z for f in initial},
            "extra_wait_s": {f.request.name: final[f.request.name].departure - f.departure for f in initial},
            "geometry_changed": {f.request.name: final[f.request.name].route.paths != f.route.paths for f in initial},
            "length_changes_m": {f.request.name: final[f.request.name].route.length - f.route.length for f in initial},
            "route_attempts": dict(result.route_attempts)}


def add_coordination_arguments(parser):
    parser.add_argument("--relative-gap", type=float, default=.03,
                        help="decentralized search: stop within this fraction of the length lower bound; 0 = full budget")
    parser.add_argument("--coordination", choices=("decentralized", "centralized"), default="decentralized")
    parser.add_argument("--network-loss", type=float, default=0.1)
    parser.add_argument("--network-delay-ms", type=float, default=150)
    parser.add_argument("--network-min-delay-ms", type=float, default=0)
    parser.add_argument("--network-seed", type=int, default=17)
    parser.add_argument("--planning-window", type=float, default=None,
                        help="agreement deadline: 5 s normally, 15 s for mountain_exchange_5")
    parser.add_argument("--route-attempts", type=int, default=3)


def coordinate_fleet(args, world, requests, *, allow_partial=False):
    if args.coordination == "centralized":
        return plan_fleet(world, requests, separation=args.separation, max_delay=args.max_delay,
                          iterations=args.iterations, seed=args.seed), None
    network = NetworkSettings(minimum_delay=args.network_min_delay_ms / 1000,
                              maximum_delay=args.network_delay_ms / 1000,
                              loss=args.network_loss, seed=args.network_seed)
    try:
        window = args.planning_window if args.planning_window is not None else SCENARIO_PLANNING_WINDOWS.get(args.scenario, 5.0)
        result = simulate_decentralized(world, requests, separation=args.separation,
                                    max_delay=args.max_delay, iterations=args.iterations, seed=args.seed,
                                    planning_window=window, network=network,
                                    relative_gap=getattr(args, 'relative_gap', 0.0),
                                    max_route_attempts=args.route_attempts)
    except DecentralizedPlanningError as error:
        if not allow_partial or error.result is None:
            raise
        result = error.result
        print(f"PARTIAL / TIMEOUT: {error}")
    status = "agreement" if result.agreement_complete else "stopped without full agreement"
    print(f"Decentralized: {result.wall_ms:.2f} ms wall; {status} at sim_time={result.negotiation_time:.3f} s; "
          f"messages={result.messages_sent}, lost={result.messages_lost}")
    return result.plan, result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=SCENARIOS, default="hills")
    parser.add_argument("--backend", choices=("cpp", "python", "auto"), default="cpp")
    parser.add_argument("--iterations", type=int, default=600)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--separation", type=float, default=40)
    parser.add_argument("--max-delay", type=float, default=120)
    parser.add_argument("--output", type=Path, default=Path("outputs/fleet"))
    add_coordination_arguments(parser)
    args = parser.parse_args()
    os.environ["DUBINS_BACKEND"] = args.backend
    try:
        print(f"Backend: {backend_name()}")
        world, requests = make_fleet_scenario(args.scenario)
        plan, protocol = coordinate_fleet(args, world, requests)
        print_schedule(plan)
        args.output.mkdir(parents=True, exist_ok=True)
        for f in plan.flights:
            stem = args.scenario + ("_decentralized" if protocol else "")
            target = args.output / f"{stem}_{f.request.name}.csv"
            with target.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(("time_s", "east_m", "north_m", "altitude_m", "roll_rad", "pitch_rad", "yaw_rad"))
                writer.writerows(sample_flight(f, plan.horizon))
            print(target)
        if protocol:
            target = args.output / f"{args.scenario}_decentralized.protocol.json"
            target.write_text(json.dumps(protocol.metadata(), ensure_ascii=False, indent=2), encoding="utf-8")
            print(target)
    except (ValueError, RuntimeError) as error:
        parser.exit(2, f"{error}\n")


if __name__ == "__main__":
    main()
