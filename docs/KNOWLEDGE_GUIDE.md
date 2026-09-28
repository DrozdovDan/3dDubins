# Method and literature guide

The phrase “3D Dubins” covers several different aircraft models. Before choosing code, decide which limits and endpoint conditions your aircraft must satisfy. A delivery route also needs a separate decision about the order of stops and a separate check against terrain, weather, and restricted airspace.

## 1. Four layers of an aerial delivery route

| Layer | Question | Papers and code in this collection |
|---|---|---|
| Mission routing | Which depot, delivery point, or transfer point comes next? | [2017 data collection/orienteering](https://comrob.fel.cvut.cz/papers/ecmr17dop3d.pdf); [2023 visual inspection tours](https://arxiv.org/abs/2301.05309) shows another multi-target formulation. |
| Local steering | What flyable curve connects two specified position/heading states? | [2007 Dubins airplane](https://msl.cs.uiuc.edu/~lavalle/papers/ChiLav07b.pdf); [Owen–Beard–McLain](https://scholarsarchive.byu.edu/facpub/1900/); [2020 bounded-curvature/pitch method](https://comrob.fel.cvut.cz/papers/icra20dubins3d.pdf). The accompanying [`core.py`](../dubins_airplane/core.py) implements Owen's Section 4 construction. |
| Global path and safety | Can the curve avoid terrain and no-fly areas? | [2024 steep-terrain navigation](https://arxiv.org/abs/2401.04831); [Hybrid A* example](https://github.com/zgoddard3/hybrid-astar); [OMPL 3D Dubins spaces](https://ompl.kavrakilab.org/spaces.html). |
| Tracking and operations | Can the real aircraft follow the curve in wind and with its actual performance? | [Owen–Beard–McLain guidance](https://sector3.imm.uran.ru/magistr/literat/BeardMcLain__.pdf); [2024 steady-wind paths](https://arxiv.org/abs/2412.04797); [2026 smoothing](https://arxiv.org/abs/2603.21713). |

The geometry of a route leg does not assign deliveries, detect obstacles, or command an autopilot. It provides a candidate curve and a distance estimate for those other layers.

## 2. Which “3D Dubins” model is needed?

| Model | Endpoint state and main limits | Practical choice |
|---|---|---|
| Dubins airplane, with helical turns | `(x, y, z, yaw)`; horizontal turn radius and maximum climb/descent angle | Good starting point for fixed-wing delivery legs when arrival pitch is free. Use this repository's Python example, [ntnu-arl/DubinsAirplane](https://github.com/ntnu-arl/DubinsAirplane), or OMPL's `OwenStateSpace`. |
| Bounded spatial curvature and pitch | Usually `(x, y, z, yaw, pitch)`; true 3D curvature and pitch bounds | Use [comrob/Dubins3D.jl](https://github.com/comrob/Dubins3D.jl), or its [Rust reimplementation](https://github.com/Rylandl/dubins3d.rs), if endpoint pitch matters. |
| Generalized pitch/yaw-rate model | More detailed orientation and separate pitch/yaw control limits | See the [2025 generalized model](https://arxiv.org/abs/2509.24143) and [author's Python code](https://github.com/DeepakPrakashKumar/3D-Motion-Planning-for-Generalized-Dubins-with-Pitch-Yaw-constraints). |
| Spatial CSC bounded-curvature path | Full 3D endpoint tangent directions; curve–straight–curve family | See the [2024 analytic CSC paper](https://arxiv.org/abs/2405.08710) and [Mathematica code](https://github.com/aabecker/dubins3D). It is a different boundary-value problem from the simple Dubins airplane. |

These methods are not interchangeable: a heading-only endpoint does not prescribe approach climb angle, and a bound on the horizontal turn radius differs from a bound on full 3D curvature.

## 3. How the included Python planner works

Input is a pair of `Pose(x, y, z, yaw)` states, a minimum horizontal turn radius `R`, and a maximum absolute flight-path angle `gamma_max`. All positions and `R` are in metres; angles are radians. `x` points east, `y` north, and `z` upward in the example.

This is an independent implementation of [Owen–Beard–McLain, Section 4](https://www2.et.byu.edu/~beard/papers/preprints/BeardMcLain__.pdf). Unlike the previous six-word/full-turn approximation, it uses the chapter's four **CSC** families: `RSR`, `RSL`, `LSR`, `LSL`. First choose the shortest at `R_min` and call its horizontal length `L0`. The required horizontal distance at the limiting angle is `D = abs(delta_z) / tan(gamma_max)`.

| Case | Construction in the code | Chapter |
|---|---|---|
| `low`: `D <= L0` | Keep `R_min`; set `gamma = atan2(delta_z, L0)` | 4.2.1 |
| `medium`: `L0 < D < L0 + 2*pi*R_min` | Advance on the start circle for climb (backwards on the end circle for descent), re-plan a CSC to/from this intermediate pose, and bisect `phi` until `phi*R_min + L_car(intermediate) = D` | 4.2.3 |
| `high`: `D >= L0 + 2*pi*R_min` | Set `k = floor((D-L0)/(2*pi*R_min))`; bisect `R* >= R_min` until `L_car(R*) + 2*pi*k*R* = D` | 4.2.2, Equation (20) |

The equality at the medium/high boundary is represented as one full turn at `R_min`; the two constructions have the same limiting length. Medium/high fly at `+/-gamma_max` within numerical tolerance. Complete high-case turns are folded into the start helix for climb and the end helix for descent. Height changes linearly with distance along the horizontal projection, including all arcs and the straight. The 3D length is `hypot(L_xy, delta_z)`.

`path.altitude_case` identifies the case; `base_word` is the CSC chosen at `R_min`; `word` describes the fitted construction (three letters for low/high, four for medium). `path.radius` is **R***, not necessarily the input minimum; `minimum_radius` keeps the input value. `extra_turns` counts added complete turns in the high case. `extension_angle` is the displacement parameter `phi` used in the medium solve, not necessarily the sweep of the intermediate opposite-turn arc. Equation (19) is available as `minimum_turn_radius(airspeed, max_bank_angle, gravity=9.81)`.

Numerical detail: `L_car` is re-evaluated as the shortest feasible CSC whenever the trial radius or intermediate pose changes, as defined in the chapter. The [NTNU code](https://github.com/ntnu-arl/DubinsAirplane) freezes some turn families during searches; results can differ where that family switches or its wrapped arc jumps. This implementation rejects discontinuities as false roots and bounds all searches instead of risking an infinite loop. It follows the geometric equations rather than reproducing those numerical quirks.

### Run it

From the repository root, with Python 3.10 or newer:

```bash
python -m examples.example
python -m unittest discover -s tests -v
```

`example.py` prints two high-case route-leg summaries and writes `outputs/example_route.csv` with sampled coordinates. For a 3D plot, run `python -m pip install -r requirements.txt`, then `python -m examples.example --plot`. The planner and CSV example themselves use only the Python standard library.

Three further runnable examples isolate common questions: [`example_arrival_heading.py`](../examples/example_arrival_heading.py) compares approach headings, [`example_climb_limit.py`](../examples/example_climb_limit.py) compares the three altitude cases, and [`example_delivery_mission.py`](../examples/example_delivery_mission.py) connects fixed waypoints and exports `outputs/delivery_mission_route.csv`. It does not choose waypoint order or model landing or payload handoff.

For labeled top-down, 3D, and altitude views of every example, plus route animations, see [`VISUAL_GUIDE.md`](VISUAL_GUIDE.md). Every example accepts `--plot` or `--save-visuals`.

Smallest API example:

```python
from math import radians
from dubins_airplane import Pose, plan_dubins_airplane

depot = Pose(0, 0, 120, radians(0))
customer = Pose(500, 250, 250, radians(90))
path = plan_dubins_airplane(depot, customer,
                            radius=60,
                            max_flight_path_angle=radians(15))
print(path.altitude_case, path.word, path.radius, path.extra_turns, path.length)
points = path.sample(max_horizontal_step=5)
print(points[-1])  # customer's x, y, z, yaw (modulo 2*pi)
```

### Guarantees and limits of the example

The constructed path reaches the requested position and heading (within floating point error), travels forward, uses horizontal turns of radius at least `R_min`, and stays within `abs(gamma_max)` to numerical tolerance. Tests cover the three altitude cases, both directions of altitude change, case boundaries, join tangents, all four CSC base families, and 200 randomized separated endpoints. Sixteen offline NTNU example values check length and radius within that reference's numerical accuracy, without downloading or executing third-party code during tests.

The chapter's CSC construction does not include `RLR`/`LRL`, so it is not a general six-family Dubins solver. The NTNU examples conservatively require at least `6*R_min` horizontal separation. Some closer configurations still work, but an unsolvable length equation raises `ValueError`. A zero-length identical pose is supported; an arbitrary small climb above the same horizontal pose is not silently replaced with a shallow full circle. No globally shortest path claim is made over all possible 3D trajectories.

It assumes no wind and a constant flight-path angle over each leg. Endpoint pitch, pitch rate, bank transients, speed evolution, fuel, battery, and payload are not modeled. Section 3's vector-field guidance and Section 4.3's flight-state path manager are not implemented: `pose_at`/`sample` evaluate the geometric reference curve. Obstacles and airspace require a separate planner.

## 4. What each paper contributes

| Year | Paper | Contribution to the decision |
|---:|---|---|
| 2007 | [Chitsaz–LaValle, Time-Optimal Paths for a Dubins Airplane](https://msl.cs.uiuc.edu/~lavalle/papers/ChiLav07b.pdf) | Theoretical baseline for altitude-aware Dubins-airplane paths. |
| 2010 | [Hota–Ghose, Optimal Geometrical Path in 3D](https://sector3.imm.uran.ru/magistr/literat/05653663.pdf) | Spatial curvature-constrained geometric construction; not the same state as a heading-only airplane. |
| 2013/2014 | [Owen–Beard–McLain, Implementing Dubins Airplane Paths](https://sector3.imm.uran.ru/magistr/literat/BeardMcLain__.pdf) | Fixed-wing model, low/medium/high altitude cases, helix construction, and guidance. Main reference for this example. |
| 2016 | [Hota–Ghose, Constrained Pitch and Yaw](https://www.cambridge.org/core/services/aop-cambridge-core/content/view/0BF09514B974A55F4B571BE7942255F6/S026357471600076Xa.pdf/novel-three-dimensional-optimal-path-planning-method-for-vehicles-with-constrained-pitch-and-yaw.pdf) | Separate pitch/yaw constrained 3D trajectories. |
| 2017 | [Váňa et al., Data Collection with Dubins Airplane](https://comrob.fel.cvut.cz/papers/ecmr17dop3d.pdf) | Connects Dubins-airplane steering with a travel-budget mission problem. |
| 2020 | [Váňa et al., Minimal 3D Dubins Path](https://comrob.fel.cvut.cz/papers/icra20dubins3d.pdf) | Fast decoupled horizontal/vertical method and path-quality bounds. |
| 2021 | [Váňa et al., Non-linear Optimization](https://comrob.fel.cvut.cz/papers/ecmr21dubins3d-nlp.pdf) | Numerical benchmark for pitch-constrained spatial solutions. |
| 2022 | [Park, 3D Dubins Path Smoothing](https://mdpi-res.com/d_attachment/applsci/applsci-12-11336/article_deploy/applsci-12-11336-v2.pdf) | Makes transitions smoother with Bézier curves for waypoint-following use. |
| 2023 | [Hague et al., Visual Inspection Tours](https://arxiv.org/abs/2301.05309) | Multi-target aerial tour planning with Dubins-airplane edges. |
| 2024 | [Baez et al., Analytic 3D CSC](https://arxiv.org/abs/2405.08710) | Analytic CSC solutions between oriented 3D endpoints. |
| 2024 | [Lim et al., Safe Low-Altitude Navigation](https://arxiv.org/abs/2401.04831) | Global fixed-wing planning in terrain using a Dubins-airplane space. |
| 2024 | [Dubins Airplane in Steady Wind](https://arxiv.org/abs/2412.04797) | Wind changes the minimum-time routing question. |
| 2025 | [Reparametrization of 3D CSC Dubins Paths](https://arxiv.org/abs/2503.11560) | Reduces the search dimensions for the CSC family. |
| 2025 | [Generalized Dubins with Pitch/Yaw Rate Constraints](https://arxiv.org/abs/2509.24143) | More detailed aircraft-relevant control bounds. |
| 2026 | [Simple Trajectory Smoothing for UAV Reference Paths](https://arxiv.org/abs/2603.21713) | A smoothing layer after route generation. |

For implementation options and direct Git links, see [IMPLEMENTATIONS.md](IMPLEMENTATIONS.md). For the full paper catalogue and two additional aerial papers, see [papers/README.md](../papers/README.md).
