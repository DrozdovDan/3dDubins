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

Without CMake, GCC/Clang can compile the executable directly:

```bash
g++ -std=c++17 -O3 -fno-fast-math -ffp-contract=off native/example.cpp native/core.cpp native/terrain.cpp native/rrt_star.cpp native/bindings.cpp -o native/build/example_cpp
```

Create `native/build` first. [`example.cpp`](example.cpp) demonstrates both
an Owen climb and a safe RRT* route around a building. Its C++ RNG differs
from CPython's, so it is reproducible but not the Python example's exact route.

## API and responsibilities

Include [`dubins.hpp`](dubins.hpp), then call `dubins::plan` or
`dubins::rrt_star`. `Path::pose_at` takes **horizontal** distance in metres.
Coordinates are east/north/up; yaw and flight-path angles are radians.
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

## Why it is faster without approximating the planner

- Fixed-size paths and reserved contiguous nodes avoid per-steer Python objects.
- Steering, sorting, rewiring and conservative interval checks stay in C++.
- Euclidean distance is only a **lower bound**: a connection is skipped only
  when it cannot affect the exact directed nearest/parent selection or improve
  a rewire. The bound includes numerical endpoint tolerance. It never becomes
  a straight-line aircraft edge or an approximate nearest-neighbour metric.
- Parent candidates are evaluated lazily in cost-bound order. An exact
  feasible neighbour makes a separate nearest search unnecessary; otherwise
  the original single-nearest fallback is preserved. A blocked nearest is
  never replaced by an arbitrary farther free node.
- Collision checks cache segment prefixes and sin/cos, evaluating the same
  continuous curve. Interval endpoint heights use the same linear expression
  without redundant XY walks; thin obstacles and subdivision bounds remain.
- Root scans reuse the exact shared endpoint values of adjacent brackets.
  Goal sorting reuses buffers and sorts indices rather than whole paths.
- All Owen branches, bounded root searches and continuous collision
  subdivision tolerances remain the same as the Python reference.

Run `python -m unittest discover -s tests -v` after building to include the
cross-language tests, and `python -m examples.benchmark_planner --backend both`
to compare the same seed/iteration budgets. Different math libraries may
resolve floating-point ties differently. This does not add an optimality or
flight-safety guarantee to the original finite-budget, CSC-only planner.

To verify the current full-search timing target on your machine:

```bash
python -m examples.benchmark_planner --backend cpp --iterations 600 --repeats 50 --max-ms 100
```

The benchmark exits nonzero for any call taking 100 ms or longer, **without
truncating the search**. Map construction and visualization are excluded;
Python preparation and native input/output conversion are included. The
reported observed maxima are not worst-case real-time bounds on Windows.
