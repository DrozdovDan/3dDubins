"""Paired native optimization check: identical inputs, full budgets, exact paths.

python -m examples.benchmark_native_speed --baseline outputs/optimization_20261027/_native_baseline.dll
Default mode checks isolated local searches; --missions measures coordination.
Neither mode silently replaces the full swarm benchmark.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import platform
from statistics import median
from time import perf_counter

from dubins_airplane import native, NoPathError, plan_safe_rrt_star
from dubins_airplane.decentralized import holding_flight
from dubins_airplane.traffic import plan_safe_rrt_star_traffic
from examples.benchmark_scenarios import (TOPOLOGIES, DIRECTIONS, SCENARIOS,
                                         make_scene, make_requests, measure_task)


def compare_search(world, request, options, libraries, traffic=False, reverse=False):
    planner = plan_safe_rrt_star_traffic if traffic else plan_safe_rrt_star
    outputs, times = {}, {}
    previous = native._library
    try:
        for label in reversed(libraries) if reverse else libraries:
            native._library = libraries[label]
            start = perf_counter()
            try:
                outputs[label] = planner(world, request.start, request.goal, request.radius,
                                         request.max_angle, **options)
            except NoPathError as error:
                outputs[label] = str(error)
            times[label] = 1000*(perf_counter()-start)
    finally:
        native._library = previous
    a, b = outputs['baseline'], outputs['optimized']
    equal = a == b  # All paths/segments, history, iterations, nodes and rewires.
    result = dict(baseline_ms=times['baseline'], optimized_ms=times['optimized'],
                  speedup=times['baseline']/times['optimized'], exact_match=equal,
                  succeeded=not isinstance(b, str),
                  length_m=b.length if not isinstance(b, str) else None,
                  iterations=b.iterations if not isinstance(b, str) else options['iterations'])
    if not equal:
        result['baseline_result'] = asdict(a) if not isinstance(a, str) else a
        result['optimized_result'] = asdict(b) if not isinstance(b, str) else b
    return result, a


def compare_mission(world, requests, topology, scenario, direction, libraries, reverse=False,
                    iterations=600, retry_iterations=3000, relative_gap=0.0):
    """Same mission/settings, but readiness/network event order may change with speed."""
    outputs = {}
    previous = native._library
    try:
        for label in reversed(libraries) if reverse else libraries:
            native._library = libraries[label]
            run = measure_task(world, requests, topology, scenario, direction=direction,
                               iterations=iterations, retry_iterations=retry_iterations,
                               relative_gap=relative_gap if label == 'optimized' else 0.0)
            outputs[label] = {key: run[key] for key in
                              ('complete', 'safety_valid', 'wall_ms', 'wall_per_agent_ms',
                               'mean_length_m', 'rrt_calls', 'replan_calls', 'calls',
                               'extra_departure_delay_s', 'total_iteration_budget')}
    finally:
        native._library = previous
    return dict(topology=topology, scenario=scenario, direction=direction, agents=len(requests),
                **outputs, speedup=outputs['baseline']['wall_ms']/outputs['optimized']['wall_ms'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--topology', nargs='+', choices=TOPOLOGIES, default=list(TOPOLOGIES))
    parser.add_argument('--direction', nargs='+', choices=DIRECTIONS, default=list(DIRECTIONS))
    parser.add_argument('--iterations', nargs='+', type=int, default=[600, 3000])
    parser.add_argument('--missions', action='store_true', help='compare entire coordinated missions instead of isolated calls')
    parser.add_argument('--relative-gap', type=float, default=0.0,
                        help='missions only: adaptive quality target for optimized library; baseline stays fixed-budget')
    parser.add_argument('--agents', nargs='+', type=int, default=[5])
    parser.add_argument('--scenario', choices=SCENARIOS, default='reconfigure')
    parser.add_argument('--agent-index', type=int, choices=(0, 1), default=0,
                        help='which of the two agents supplies isolated-search inputs')
    parser.add_argument('--output', type=Path, default=Path('outputs/benchmark_native_speed/results.json'))
    args = parser.parse_args(argv)
    if not args.iterations or any(n <= 0 for n in args.iterations):
        parser.error('iteration budgets must be positive')
    if not 0 <= args.relative_gap <= 1 or (args.relative_gap and not args.missions):
        parser.error('--relative-gap must be in [0, 1] and requires --missions')
    os.environ['DUBINS_BACKEND'] = 'cpp'
    paths = dict(baseline=args.baseline.resolve(), optimized=native.library_path().resolve())
    libraries = {label: native._load(str(path)) for label, path in paths.items()}
    report = dict(kind=('paired whole missions; event order can change with readiness time' if args.missions else
                        'paired isolated native searches, not swarm wall time'),
                  python=platform.python_version(), platform=platform.platform(),
                  configuration=dict(topologies=args.topology, directions=args.direction,
                                     scenario=args.scenario, agents=args.agents if args.missions else [2],
                                     agent_index=args.agent_index,
                                     budgets=args.iterations, fixed_seed=7,
                                     optimized_relative_gap=args.relative_gap,
                                     traffic_fixture='one permanent peer hold' if not args.missions else None),
                  source_sha256={name: hashlib.sha256(Path(name).read_bytes()).hexdigest() for name in
                                 ('native/core.cpp', 'native/rrt_star.cpp', 'native/terrain.cpp',
                                  'native/fleet.cpp', 'dubins_airplane/decentralized.py')},
                  libraries={label: dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                             for label, path in paths.items()}, runs=[])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for topology in args.topology:
        world = make_scene(topology, args.scenario)
        for direction in args.direction:
            if args.missions:
                for count in args.agents:
                    requests = make_requests(world, args.scenario, count, direction)
                    row = compare_mission(world, requests, topology, args.scenario, direction, libraries,
                                          reverse=len(report['runs']) % 2 == 1,
                                          iterations=min(args.iterations), retry_iterations=max(args.iterations),
                                          relative_gap=args.relative_gap)
                    report['runs'].append(row)
                    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
                    print(f"{topology}/{direction}/n{count}: "
                          f"{row['baseline']['wall_ms']:.1f} -> {row['optimized']['wall_ms']:.1f} ms "
                          f"({row['speedup']:.2f}x), complete={row['optimized']['complete']}, "
                          f"safe={row['optimized']['safety_valid']}", flush=True)
                continue
            requests = make_requests(world, args.scenario, 2, direction)
            request = requests[args.agent_index]
            for iterations in args.iterations:
                options = dict(iterations=iterations, seed=(1016 if iterations > 600 else 7)+args.agent_index)
                for traffic in (False, True):
                    if traffic:
                        if isinstance(baseline_route, str):
                            continue
                        options.update(start_pose=baseline_route.paths[0].start, departure=100,
                                       speed=request.speed, reservations=(holding_flight(requests[1-args.agent_index]),),
                                       separation=40, horizon=600)
                    row, route = compare_search(world, request, options, libraries, traffic,
                                                reverse=len(report['runs']) % 2 == 1)
                    if not traffic:
                        baseline_route = route
                    row.update(topology=topology, scenario=args.scenario, direction=direction, budget=iterations,
                               mode='traffic' if traffic else 'static', seed=options['seed'])
                    report['runs'].append(row)
                    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
                    print(f"{topology}/{direction}/{iterations}/{row['mode']}: "
                          f"{row['baseline_ms']:.1f} -> {row['optimized_ms']:.1f} ms "
                          f"({row['speedup']:.2f}x), exact={row['exact_match']}", flush=True)
    rows = report['runs']
    if args.missions:
        print(f"Complete safe missions: "
              f"{sum(r['optimized']['complete'] and r['optimized']['safety_valid'] for r in rows)}/{len(rows)}; "
              f"median paired speedup: {median(r['speedup'] for r in rows):.2f}x")
        return 0 if all(r['optimized']['complete'] and r['optimized']['safety_valid'] for r in rows) else 1
    print(f"Exact: {sum(r['exact_match'] for r in rows)}/{len(rows)}; "
          f"median paired speedup: {median(r['speedup'] for r in rows):.2f}x")
    return 0 if all(r['exact_match'] for r in rows) else 1


if __name__ == '__main__':
    raise SystemExit(main())
