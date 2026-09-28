# 3D Dubins for aerial logistics

A small Python implementation of the geometric planner in **Owen, Beard & McLain (2014), Section 4**. Give it start/goal positions and headings, a minimum turn radius, and a maximum climb angle. The core needs no third-party packages.

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
- [Visual guide](docs/VISUAL_GUIDE.md), [method and paper guide](docs/KNOWLEDGE_GUIDE.md), [paper sources](papers/README.md), and [other implementations](docs/IMPLEMENTATIONS.md).

Source: [the chapter](https://scholarsarchive.byu.edu/facpub/1900/) and [NTNU reference code](https://github.com/ntnu-arl/DubinsAirplane). This implements the chapter's trajectory construction, not its guidance controller. Like the chapter, the base families are `RSR`, `RSL`, `LSR`, `LSL`; `RLR`/`LRL` are excluded. Nearby poses can require those families or have no solution on this construction; the reference examples use a conservative horizontal separation of at least `6 * minimum_radius`. Unsupported length equations raise `ValueError` rather than returning the old full-turn approximation.

It does not check obstacles, airspace, wind, landing, or vehicle-specific dynamics, and is not a general globally shortest 3D solver. Local PDFs in `papers/` are Git-ignored until redistribution rights are checked.
