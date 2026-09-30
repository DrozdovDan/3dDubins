"""Four readable examples of safe Dubins RRT* planning.

python -m examples.example_obstacles --scenario buildings --plot
python -m examples.example_obstacles --scenario all --save-visuals
Add --plot or --save-visuals to see the terrain, obstruction and final route.
No hand-picked intermediate waypoints: the RRT* tree discovers the detour.
"""

import argparse
import csv
from dataclasses import dataclass
from math import degrees, exp, radians
from pathlib import Path

from dubins_airplane import (BoxObstacle, ElevationMap, FlightWorld, NoPathError,
                            LoiterCircle, plan_dubins_airplane, plan_safe_rrt_star)

SCENARIOS = {
    "hill_box": ("Обход холма и запретного объёма",
                 "Высокий объём нельзя перелететь в заданном коридоре высоты."),
    "buildings": ("Три препятствия на ровной местности",
                  "Обходим здания; рельеф ровный, поэтому видны именно горизонтальные манёвры."),
    "ridge": ("Обход горного хребта",
              "Искусственных препятствий нет: прямой путь нарушает запас над рельефом."),
    "climb": ("Перелёт на возвышенность",
              "Цель выше старта; маршрут набирает высоту внутри разрешённого коридора."),
}


@dataclass(frozen=True)
class Scenario:
    name: str
    world: FlightWorld
    start: LoiterCircle
    goal: LoiterCircle
    radius: float = 40
    max_angle: float = radians(15)

    @property
    def output_stem(self):
        return "obstacle_route" if self.name == "hill_box" else f"obstacle_{self.name}"


def make_world():
    terrain = ElevationMap.from_function(
        -180, -400, 20, 59, 40,
        lambda x, y: 65 * exp(-((x - 400) / 140)**2 - (y / 190)**2),
    )
    # An explicit forbidden volume cannot be overflown within this height band.
    obstacle = BoxObstacle(320, -130, 0, 480, 130, 300)
    return FlightWorld(terrain, min_clearance=40, max_clearance=110,
                       obstacles=(obstacle,), obstacle_margin=5)


def make_scenario(name="hill_box"):
    """Scene data only; intermediate states are always found by RRT*."""
    if name not in SCENARIOS:
        raise ValueError(f"unknown scenario: {name}")
    if name == "hill_box":
        world = make_world()
    elif name == "buildings":
        terrain = ElevationMap.from_function(-180, -400, 20, 59, 40, lambda x, y: 0)
        world = FlightWorld(terrain, 40, 110, obstacles=(
            BoxObstacle(220, -180, 0, 330, 90, 240),
            BoxObstacle(470, -60, 0, 580, 220, 240),
            BoxObstacle(650, -240, 0, 720, -80, 240),
        ), obstacle_margin=5)
    elif name == "ridge":
        terrain = ElevationMap.from_function(
            -180, -500, 20, 59, 50,
            lambda x, y: 170 * exp(-((x - 400) / 65)**2 - (y / 230)**2),
        )
        world = FlightWorld(terrain, 30, 100)
    else:  # climb: a broad slope, with safe level loiters at both ends
        terrain = ElevationMap.from_function(
            -180, -400, 20, 59, 40,
            lambda x, y: 120 / (1 + exp(-(x - 450) / 110)),
        )
        world = FlightWorld(terrain, 40, 110)
    return Scenario(name, world, world.choose_loiter(0, 0, 40),
                    world.choose_loiter(800, 0, 40))


def run_scenario(scenario, args):
    world, start, goal = scenario.world, scenario.start, scenario.goal
    title, description = SCENARIOS[scenario.name]
    print(f"\n{scenario.name}: {title}")
    print(description)
    print(f"Старт: z={start.z:.1f} м; цель: z={goal.z:.1f} м. Поиск RRT*...")
    result = plan_safe_rrt_star(world, start, goal, scenario.radius, scenario.max_angle,
                                iterations=args.iterations, seed=args.seed)
    # A fair comparison: use the SAME departure/arrival states as the found
    # route, not arbitrary and different phases of the two loiter circles.
    direct = plan_dubins_airplane(result.paths[0].start, result.paths[-1].goal,
                                  scenario.radius, scenario.max_angle)
    print(f"Прямой Dubins-путь: {'свободен' if world.path_is_free(direct) else 'заблокирован'}")
    print(f"Найдено {len(result.paths)} участков, {result.length:.1f} м; "
          f"узлов: {result.nodes}, переподключений: {result.rewires}, seed={args.seed}")
    for i, path in enumerate(result.paths, 1):
        print(f"  {i}: {path.word}, {path.length:.1f} м, "
              f"угол={degrees(path.flight_path_angle):.1f}°")
    if not all(world.path_is_free(path) for path in result.paths):
        raise RuntimeError("planned route failed its final collision check")
    root = Path(__file__).resolve().parents[1]
    output = root / "outputs" / f"{scenario.output_stem}.csv"
    output.parent.mkdir(exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("east_m", "north_m", "altitude_m", "yaw_deg"))
        for pose in result.sample(5):
            writer.writerow((pose.x, pose.y, pose.z, degrees(pose.yaw)))
    print(f"CSV: {output}")
    if args.plot or args.save_visuals:
        from dubins_airplane.visualization import RouteLeg, Waypoint, visualize_routes

        visualize_routes(
            [RouteLeg(f"Участок {i}", path) for i, path in enumerate(result.paths, 1)],
            [Waypoint("Старт", result.paths[0].start), Waypoint("Финиш", result.paths[-1].goal)],
            title, description=description,
            png=root / "visuals" / f"{scenario.output_stem}.png" if args.save_visuals else None,
            gif=root / "visuals" / f"{scenario.output_stem}.gif" if args.save_visuals else None,
            show=args.plot, world=world, loiters=(start, goal), reference_path=direct,
        )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=(*SCENARIOS, "all"), default="hill_box")
    parser.add_argument("--iterations", type=int, default=600)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--save-visuals", action="store_true")
    args = parser.parse_args()
    for name in SCENARIOS if args.scenario == "all" else (args.scenario,):
        try:
            run_scenario(make_scenario(name), args)
        except NoPathError as error:
            parser.exit(2, f"{name}: {error}\n")


if __name__ == "__main__":
    main()
