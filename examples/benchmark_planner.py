"""Reproducible, plot-free timing of the terrain-aware RRT* planner.

Run from the project root:
    python -m examples.benchmark_planner
    python -m examples.benchmark_planner --scenario ridge --iterations 200 --repeats 5

The terrain, clearance grid, and safe loiters are prepared before timing.
The measured call includes loiter validation, Dubins steering, collision
checks, search, rewiring, and reconstruction, but no CSV or visualization.
"""

from __future__ import annotations

import argparse
import os
import platform
from statistics import median
from time import perf_counter

from dubins_airplane import NoPathError, plan_safe_rrt_star
from examples.example_obstacles import SCENARIOS, make_scenario


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
    args = parser.parse_args()
    if args.iterations <= 0 or args.repeats <= 0:
        parser.error("--iterations and --repeats must be positive")

    print(f"Python {platform.python_version()}, {platform.system()} {platform.release()}, "
          f"logical CPUs: {os.cpu_count()}")
    iteration_word = "iteration" if args.iterations == 1 else "iterations"
    run_word = "run" if args.repeats == 1 else "runs"
    print(f"RRT*: {args.iterations} {iteration_word}, {args.repeats} {run_word}, "
          f"seeds {args.seed}..{args.seed + args.repeats - 1}")
    print("Timing excludes map construction, CSV, plotting and GIF encoding.", flush=True)
    print("scenario       success  first median [min..max] s  "
          "full median [min..max] s  first iteration  length median m", flush=True)

    names = SCENARIOS if "all" in args.scenario else dict.fromkeys(args.scenario)
    for name in names:
        scene = make_scenario(name)
        # Warm up the same planner code, without requiring that 20 iterations
        # already find a path in the difficult scenarios.
        measure(scene, min(args.iterations, 20), args.seed - 1)
        samples = []
        for offset in range(args.repeats):
            samples.append(measure(scene, args.iterations, args.seed + offset))
            print(f"  {name}: run {offset + 1}/{args.repeats} done", flush=True)
        first = [sample[0] for sample in samples if sample[0] is not None]
        full = [sample[1] for sample in samples]
        first_iteration = [sample[2] for sample in samples if sample[2] is not None]
        lengths = [sample[3] for sample in samples if sample[3] is not None]

        def duration(values):
            return (f"{median(values):.3f} [{min(values):.3f}..{max(values):.3f}]"
                    if values else "n/a")

        first_iter_text = f"{median(first_iteration):.0f}" if first_iteration else "n/a"
        length_text = f"{median(lengths):.1f}" if lengths else "n/a"
        print(f"{name:<14} {len(first):>2}/{args.repeats:<2}    "
              f"{duration(first):<28} {duration(full):<28} "
              f"{first_iter_text:>5}            {length_text}", flush=True)


if __name__ == "__main__":
    main()
