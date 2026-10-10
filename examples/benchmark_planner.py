"""Reproducible, plot-free timing of the terrain-aware RRT* planner.

Run from the project root:
    python -m examples.benchmark_planner
    python -m examples.benchmark_planner --scenario ridge --iterations 200 --repeats 5
    python -m examples.benchmark_planner --backend both --repeats 3
    python -m examples.benchmark_planner --backend cpp --repeats 30 --max-ms 100

The terrain, clearance grid, and safe loiters are prepared before timing.
The measured call includes loiter validation, Dubins steering, collision
checks, search, rewiring, and reconstruction, but no CSV or visualization.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from math import isfinite
import os
import platform
from statistics import median
from time import perf_counter

from dubins_airplane import NoPathError, backend_name, plan_safe_rrt_star
from examples.example_obstacles import SCENARIOS, make_scenario


@contextmanager
def select_backend(name):
    previous = os.environ.get("DUBINS_BACKEND")
    os.environ["DUBINS_BACKEND"] = name
    try:
        yield backend_name()
    finally:
        if previous is None:
            os.environ.pop("DUBINS_BACKEND", None)
        else:
            os.environ["DUBINS_BACKEND"] = previous


def duration(values):
    return (f"{median(values):.3f} [{min(values):.3f}..{max(values):.3f}]"
            if values else "n/a")


def measure(scene, iterations: int, seed: int):
    """Return (first solution seconds, total seconds, first iteration, length).

    If no route is found within the iteration budget, the first-time and
    route fields are None; total seconds still reports the unsuccessful call.
    """
    first_time = None
    first_iteration = None

    def mark_first(iteration):
        nonlocal first_time, first_iteration
        first_time = perf_counter() - started
        first_iteration = iteration

    started = perf_counter()
    try:
        result = plan_safe_rrt_star(
            scene.world, scene.start, scene.goal, scene.radius, scene.max_angle,
            iterations=iterations, seed=seed, on_first_solution=mark_first,
        )
    except NoPathError:
        return None, perf_counter() - started, None, None
    elapsed = perf_counter() - started
    return first_time, elapsed, first_iteration, result.length


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=(*SCENARIOS, "all"), nargs="+", default=["all"])
    parser.add_argument("--iterations", type=int, default=600)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--seed", type=int, default=7,
                        help="first seed; later runs use consecutive seeds")
    parser.add_argument("--backend", choices=("auto", "python", "cpp", "both"),
                        default=os.environ.get("DUBINS_BACKEND", "auto"),
                        help="both compares identical seed streams and iteration budgets")
    parser.add_argument("--max-ms", type=float,
                        help="fail if any full planning call takes this many milliseconds or more; "
                             "does not stop or shorten the search")
    args = parser.parse_args()
    if args.iterations <= 0 or args.repeats <= 0:
        parser.error("--iterations and --repeats must be positive")
    if args.max_ms is not None and (not isfinite(args.max_ms) or args.max_ms <= 0):
        parser.error("--max-ms must be finite and positive")

    print(f"Python {platform.python_version()}, {platform.system()} {platform.release()}, "
          f"logical CPUs: {os.cpu_count()}")
    iteration_word = "iteration" if args.iterations == 1 else "iterations"
    run_word = "run" if args.repeats == 1 else "runs"
    print(f"RRT*: {args.iterations} {iteration_word}, {args.repeats} {run_word}, "
          f"seeds {args.seed}..{args.seed + args.repeats - 1}")
    print("Timing excludes map construction, CSV, plotting and GIF encoding.", flush=True)
    print("scenario       backend success  first median [min..max] s  "
          "full median [min..max] s  first iteration  length median m", flush=True)

    names = SCENARIOS if "all" in args.scenario else dict.fromkeys(args.scenario)
    violations = []
    backends = ("python", "cpp") if args.backend == "both" else (args.backend,)
    # Fail explicitly before timing if a requested native backend is missing.
    for backend in backends:
        with select_backend(backend):
            pass
    for name in names:
        scene = make_scenario(name)
        # Warm up the same planner code, without requiring that 20 iterations
        # already find a path in the difficult scenarios.
        measurements = {backend: [] for backend in backends}
        for backend in backends:
            with select_backend(backend):
                measure(scene, min(args.iterations, 20), args.seed - 1)
        for offset in range(args.repeats):
            # Alternate order to reduce systematic warm-cache/load bias.
            order = backends if offset % 2 == 0 else tuple(reversed(backends))
            for backend in order:
                with select_backend(backend):
                    measurements[backend].append(measure(scene, args.iterations, args.seed + offset))
            print(f"  {name}: run {offset + 1}/{args.repeats} done", flush=True)
        for backend, samples in measurements.items():
            first = [sample[0] for sample in samples if sample[0] is not None]
            full = [sample[1] for sample in samples]
            first_iteration = [sample[2] for sample in samples if sample[2] is not None]
            lengths = [sample[3] for sample in samples if sample[3] is not None]
            first_iter_text = f"{median(first_iteration):.0f}" if first_iteration else "n/a"
            length_text = f"{median(lengths):.1f}" if lengths else "n/a"
            with select_backend(backend) as actual:
                print(f"{name:<14} {actual:<7} {len(first):>2}/{args.repeats:<2}    "
                      f"{duration(first):<28} {duration(full):<28} "
                      f"{first_iter_text:>5}            {length_text}", flush=True)
            if args.max_ms is not None:
                slow = [(args.seed+i, sample[1]*1000) for i, sample in enumerate(samples)
                        if sample[1]*1000 >= args.max_ms]
                violations.extend((name, backend, seed, elapsed) for seed, elapsed in slow)
                print(f"  <{args.max_ms:g} ms: {len(samples)-len(slow)}/{len(samples)} calls; "
                      f"worst {max(full)*1000:.2f} ms", flush=True)
        if args.backend == "both":
            py, cpp = measurements["python"], measurements["cpp"]
            differences = []
            for reference, native in zip(py, cpp):
                if (reference[3] is None) != (native[3] is None):
                    raise RuntimeError("backends disagree on route existence within the same budget")
                if reference[3] is not None:
                    delta = abs(reference[3] - native[3])
                    differences.append(delta)
                    if delta > 1e-8 * max(1.0, reference[3]):
                        raise RuntimeError(f"route length mismatch: {reference[3]} vs {native[3]}")
            speedup = median(s[1] for s in py) / median(s[1] for s in cpp)
            quality = f"max length delta {max(differences):.3g} m" if differences else "no routes found"
            print(f"  C++ speedup: {speedup:.1f}x; {quality}", flush=True)
    if violations:
        print("Timing target missed (the full search still completed):", flush=True)
        for name, backend, seed, elapsed in violations:
            print(f"  {name}, {backend}, seed={seed}: {elapsed:.2f} ms", flush=True)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
