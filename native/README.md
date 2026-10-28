# C++17 backend

Both Owen steering and the whole directed RRT* search run here. Python only
validates loiters, supplies the original seeded sample stream and converts
the final result for the existing plotting/CSV API. Native planning releases
the Python GIL. Each call owns its tree and the C ABI has thread-local errors.

## Build

For Python users, from the repository root:

```bash
python -m dubins_airplane.build_native
# Or select an explicit compiler:
python -m dubins_airplane.build_native --compiler g++
```

GCC, Clang, MSVC (Developer shell) and Zig are supported. The build uses
C++17, `-O3` (MSVC `/O2`), double precision and **no fast-math**. Generated
libraries and build files are Git-ignored. Close Python processes that have
loaded the library before rebuilding it on Windows. The portable Zig build
currently targets x86-64 Windows; use a matching compiler on other targets.

For a C++ library and a standalone executable (no Python required):

```bash
cmake -S native -B native/build -DCMAKE_BUILD_TYPE=Release
cmake --build native/build --config Release
# Single-configuration generators (Linux, Ninja):
./native/build/example_cpp
# Visual Studio generator on Windows:
./native/build/Release/example_cpp.exe
# Optionally install the library beside the Python adapter:
cmake --install native/build --config Release --prefix .
```

Without CMake, GCC/Clang can compile the Python shared library directly:

```bash
g++ -std=c++17 -O3 -fno-fast-math -ffp-contract=off -shared -fPIC native/core.cpp native/terrain.cpp native/rrt_star.cpp native/fleet.cpp native/bindings.cpp -o dubins_airplane/_native.so
```

Use CMake for the standalone examples and their generated visualization assets.
[`example.cpp`](example.cpp) demonstrates both
an Owen climb and a safe RRT* route around a building. Its C++ RNG differs
from CPython's, so it is reproducible but not the Python example's exact route.

## API and responsibilities

Include [`dubins.hpp`](dubins.hpp), then call `dubins::plan` or
`dubins::rrt_star`. `Path::pose_at` takes **horizontal** distance in metres.
Coordinates are east/north/up; yaw and flight-path angles are radians.
The final optional `rrt_star` argument is `relative_gap` (default `0`, full
budget). For example, `rrt_star(world, roots, goals, samples, options, nullptr,
nullptr, 0.03)` stops when a checked route is within 3% of a physical length
lower bound for those discrete endpoints. With traffic, pass its pointer in
the preceding argument. If no route meets the target, the entire sample
budget remains available. This is a local length criterion, not joint swarm
optimality. The optional C entry point `dubins_rrt_adaptive` leaves the existing
C ABI and `Options` layout unchanged; rebuild to use the new symbol.
`Path` stores at most four segments; kind `0=L`, `1=R`, `2=S`; altitude case
`0=low`, `1=medium`, `2=high`. `base_word` indexes `RSR, RSL, LSR, LSL`.

`World` borrows row-major lower/upper flight-band arrays and closed obstacle
boxes. Keep them alive and unchanged during a call. Use `dubins_offset` for
conservative Euclidean terrain bands (`upper=1` for the minimum-clearance
floor, `upper=0` for the maximum-clearance ceiling). Unknown map space is
forbidden. The standalone caller must validate complete start/goal loiter
disks, just as the Python facade does, before supplying their tangent states.

`rrt_star` accepts zero-cost roots, terminal poses and a sample stream; an
empty `Result.paths` means no solution within the budget. `Options.time_limit`
is seconds, negative for unlimited, and checked between iterations. A first
solution callback receives its iteration (possibly zero); return nonzero to
abort. The C ABI returns status codes instead of throwing across ctypes;
buffer-size requirements are documented in the header.

The fleet adapter additionally uses `dubins::check_separation` / `dubins_fleet_check`.
`TimedFlight` borrows valid Owen path arrays, constant 3D ground speed, departure
time and start/goal circles. Keep all arrays alive for the call; ensure tangent
endpoints and continuous leg joins. Motion includes start-loiter waiting and
indefinite goal loitering. Interval subdivision certifies center distance using
the sum-of-speeds bound; unresolved intervals are rejected conservatively.
This checker does not assign priorities, plan routes, certify body margins or
implement a standalone C++ fleet scheduler: those orchestration steps are in
[`fleet.py`](../dubins_airplane/fleet.py).
The peer-to-peer simulation is orchestrated by
[`decentralized.py`](../dubins_airplane/decentralized.py): independent native
searches run concurrently (ctypes releases the GIL), with versioned messages
and local scheduling decisions in Python. No additional native rebuild is needed
when only that protocol changes.

`dubins_rrt_traffic` adds native time-aware spatial replanning; **rebuild the DLL**
for this feature. It uses one fixed root and departure, constant speed, and borrowed
peer `TimedFlight` arrays in `Traffic`. Every edge is continuously checked against
the moving peers; internal edge endpoints are exact path points, not fake loiters.
Parent changes revalidate the entire retimed subtree and goal connections before
reuse. A copied incumbent retains the best complete safe path. Initial waiting
and terminal circles are checked too. Python never makes per-edge callbacks in
the native search. This is prioritized finite-budget planning, not joint optimal
planning or post-commit online control; existing `dubins_rrt` behavior is unchanged.

## Why it is faster without approximating the planner

- Fixed-size paths and reserved contiguous nodes avoid per-steer Python objects.
- Steering, sorting, rewiring and conservative interval checks stay in C++.
- `max(Euclidean distance, |dz|/sin(max_angle))` is only a **lower bound**:
  steep climbs/descents cannot be shorter than the climb-angle limit allows.
  A connection is skipped only
  when it cannot affect the exact directed nearest/parent selection or improve
  a rewire. The bound includes numerical endpoint tolerance. It never becomes
  a straight-line aircraft edge or an approximate nearest-neighbour metric.
- Parent candidates are evaluated lazily in cost-bound order. An exact
  feasible neighbour makes a separate nearest search unnecessary; otherwise
  the original single-nearest fallback is preserved. A blocked nearest is
  never replaced by an arbitrary farther free node.
- The fallback also visits lower bounds in increasing order, retaining exact
  directed steering and original node-index tie breaking.
- High-altitude radius fitting caches the endpoint problem's bearings and
  trigonometry; the bisection brackets, residuals and tolerances are unchanged.
- Per-tree goal caches reuse exact steering geometry and static collision
  decisions for retimed nodes and repeated goal-biased samples. Duplicate
  terminal poses share their geometric cache. **Dynamic separation is always
  checked again at the current arrival time**, including retimed descendants.
- Collision checks cache segment prefixes and sin/cos, evaluating the same
  continuous curve. Interval endpoint heights use the same linear expression
  without redundant XY walks; thin obstacles and subdivision bounds remain.
- Root scans reuse the exact shared endpoint values of adjacent brackets.
  Goal sorting reuses buffers and sorts indices rather than whole paths.
- All Owen branches, bounded root searches and continuous collision
  subdivision tolerances remain the same as the Python reference.
- Each traffic search prepares immutable peer motions once. Segment prefixes
  avoid repeated trigonometry walks in distance checks. A conservative vertical
  certificate includes the original sampler's Lipschitz margin; other pairs
  retain the same continuous interval checks and tolerances.
- Terminal connections whose physical lower bound exceeds the saved incumbent
  are deferred. Rewiring revisits deferred descendants after their costs fall.
  Goal-biased samples memoise individual requested connections even before the
  full terminal cache is built. Invalid steering results are memoised as well.

Run `python -m unittest discover -s tests -v` after building to include the
cross-language tests, and `python -m examples.benchmark_planner --backend both`
to compare the same seed/iteration budgets. Different math libraries may
resolve floating-point ties differently. This does not add an optimality or
flight-safety guarantee to the original finite-budget, CSC-only planner.

For a paired comparison against a saved older library, with full budgets and
an equality check of paths, cost history and tree counters:

```bash
python -m examples.benchmark_native_speed --baseline path/to/old_native.dll --output outputs/native_speed/local.json
# End-to-end coordination: scheduling order can change with readiness times.
python -m examples.benchmark_native_speed --baseline path/to/old_native.dll --missions --agents 5 --output outputs/native_speed/missions.json
```

These are two measurements per task (one per implementation), not a sweep of
random seeds. Local static/traffic fixtures are not swarm wall time; the second
command includes real coordination and full spatial replanning. A finite-budget
search returning NoPath remains in the local results rather than being dropped.

Plot the saved paired measurements without rerunning the planner:
`python -m examples.plot_native_speed outputs/optimization_20261027`.
This creates `figures/metrics.png` (normalized mission time and path length) and
`figures/local_time.png` (isolated-search time), both in 16:9 PNG and vector SVG.

To verify the current full-search timing target on your machine:

```bash
python -m examples.benchmark_planner --backend cpp --iterations 600 --repeats 50 --max-ms 100
```

The benchmark exits nonzero for any call taking 100 ms or longer, **without
truncating the search**. Map construction and visualization are excluded;
Python preparation and native input/output conversion are included. The
reported observed maxima are not worst-case real-time bounds on Windows.
