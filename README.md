# 3D Dubins for aerial logistics

A small Python implementation of **Owen, Beard & McLain (2014)** trajectories and **Lim et al. (2024)** terrain-aware planning between safe loiter circles. The planners need no third-party packages; plotting is optional.

![Two-leg route animation](visuals/two_leg_route.gif)

## Run

From this folder:

```bash
python -m examples.example
python -m unittest discover -s tests -v
```

For plots and GIF generation:

```bash
python -m pip install -r requirements.txt
python -m examples.example --plot
python -m examples.example --save-visuals
```

The examples write CSV routes to `outputs/`. Saved PNGs and GIFs are in `visuals/`.

## Use it in Python

```python
from math import radians
from dubins_airplane import Pose, plan_dubins_airplane

start = Pose(0, 0, 120, radians(0))
goal = Pose(500, 250, 500, radians(90))
path = plan_dubins_airplane(start, goal, 60, radians(15))
print(path.altitude_case, path.word, path.radius, path.extra_turns, path.length)
points = path.sample(5)
```

`x` is east, `y` north, `z` upward (metres); yaw is in radians from east. `L`, `R`, and `S` mean left turn, right turn, and straight. `path.minimum_radius` is the input limit; `path.radius` is the actual radius.

The construction follows the chapter's three altitude cases:

- **Low:** choose the shortest of four CSC paths and fit a constant climb/descent angle.
- **Medium:** insert an additional arc and fit its displacement angle by bisection.
- **High:** add complete turns and fit a larger radius by bisection, at the maximum climb/descent angle.

The flight-path angle is constant within a leg. Extension is placed at the start for climb and at the end for descent. `minimum_turn_radius(speed, bank_angle)` also implements Equation (19); arguments are metres/second and radians.

## Explore

- [`example.py`](examples/example.py): climb to a high transfer point, then descend.
- [`example_arrival_heading.py`](examples/example_arrival_heading.py): same destination, different arrival headings.
- [`example_climb_limit.py`](examples/example_climb_limit.py): compare the low, medium, and high constructions.
- [`example_delivery_mission.py`](examples/example_delivery_mission.py): several preset delivery-area waypoints.
- [`example_obstacles.py`](examples/example_obstacles.py): four Dubins RRT* scenarios: a hill with a forbidden volume, several buildings, a mountain ridge, and a climb to higher terrain.
- [`benchmark_planner.py`](examples/benchmark_planner.py): repeatable first-solution and full-search timing for those scenarios.
- [Visual guide](docs/VISUAL_GUIDE.md), [method and paper guide](docs/KNOWLEDGE_GUIDE.md), [paper sources](papers/README.md), and [other implementations](docs/IMPLEMENTATIONS.md).

Source: [the chapter](https://scholarsarchive.byu.edu/facpub/1900/) and [NTNU reference code](https://github.com/ntnu-arl/DubinsAirplane). This implements the chapter's trajectory construction, not its guidance controller. Like the chapter, the base families are `RSR`, `RSL`, `LSR`, `LSL`; `RLR`/`LRL` are excluded. Nearby poses can require those families or have no solution on this construction; the reference examples use a conservative horizontal separation of at least `6 * minimum_radius`. Unsupported length equations raise `ValueError` rather than returning the old full-turn approximation.

## Plan around terrain and obstacles

```bash
python -m examples.example_obstacles
python -m examples.example_obstacles --plot
python -m examples.example_obstacles --save-visuals
python -m examples.example_obstacles --scenario buildings --plot
python -m examples.example_obstacles --scenario ridge --plot
python -m examples.example_obstacles --scenario climb --plot
python -m examples.example_obstacles --scenario all --save-visuals
```

To measure planner speed without plotting, CSV export, or GIF encoding:

```bash
python -m examples.benchmark_planner --iterations 600
# Optional: repeat with several seeds for timing variability.
python -m examples.benchmark_planner --scenario hill_box --iterations 600 --repeats 5
```

The benchmark reports time to the first feasible path and total RRT* call time (median and range) for each scenario. It prepares the map before timing and uses consecutive random seeds. These are wall-clock measurements on the current computer, not flight-time estimates or an optimality guarantee.

![Terrain and obstacle avoidance](visuals/obstacle_route.gif)

Read the Russian plots left to right: **map → 3D context → altitude profile**. The entire found route is blue; orange hatched areas are forbidden volumes with their height ranges labeled. Teal dashed circles are safe start/goal loiters. A direct Dubins connection between the **same endpoint states** is red dashed if blocked, grey dashed if free. A red cross locates a blocked sample. Below, blue altitude stays inside the green permitted corridor; grey-brown ground is the terrain beneath this route. Small numbers mark joins, not separate missions. GIFs synchronize the moving marker across all three panels.

The default `hill_box` scenario avoids a forbidden volume over a hill. `buildings` demonstrates lateral avoidance on flat terrain, `ridge` avoids high terrain without artificial obstacles, and `climb` reaches a higher destination with a valid direct path. Each example lets RRT* choose intermediate states; none are hand-picked. Use `--seed` and `--iterations` to change the reproducible search. The default CSV is `outputs/obstacle_route.csv`; the other stems are `obstacle_buildings`, `obstacle_ridge`, and `obstacle_climb`. PNGs and GIFs use the same stems in `visuals/`.

```python
from math import radians
from dubins_airplane import ElevationMap, FlightWorld, BoxObstacle, plan_safe_rrt_star

terrain = ElevationMap.from_function(-180, -400, 20, 59, 40, lambda x, y: 0)
world = FlightWorld(terrain, min_clearance=40, max_clearance=110,
                    obstacles=[BoxObstacle(320, -130, 0, 480, 130, 300)])
start = world.choose_loiter(0, 0, radius=40)
goal = world.choose_loiter(800, 0, radius=40)
route = plan_safe_rrt_star(world, start, goal, radius=40,
                           max_flight_path_angle=radians(15), seed=7)
points = route.sample(5)
```

Terrain is a grid of local metre heights (`heights[row][column]`), not latitude/longitude. Clearance is based on Euclidean terrain offsets, not just height above the cell below. The checker conservatively bounds whole curve intervals, so it cannot skip a thin box between point samples. Unknown map space is forbidden.

This independently implements Lim's planning idea, not the complete ROS/PX4 system. Its raster cells, finite circle samples and CSC steering are conservative approximations; a search failure is **not** a proof that no route exists. Exact differences are in the [method guide](docs/KNOWLEDGE_GUIDE.md#31-terrain-and-obstacle-planning-lim-et-al).

The standalone `plan_dubins_airplane` still ignores obstacles; use `plan_safe_rrt_star` for the supplied static map. Neither planner models wind, tracking error, aircraft size unless margins are supplied, landing, or vehicle-specific dynamics. This is not flight-certified software or a general globally shortest 3D solver. Local PDFs in `papers/` are Git-ignored until redistribution rights are checked.
