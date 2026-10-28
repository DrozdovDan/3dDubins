"""AirLogistics-3D v1: fixed-scale, seeded fleet benchmark (no visualization).

python -m examples.benchmark_fleet --runs 3 --replay-local
Protocol, sources, limitations and full evaluation commands: docs/BENCHMARK.md.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from math import exp, hypot, isclose, sin, pi
import os
from pathlib import Path
import platform
from random import Random
from statistics import median
from time import perf_counter

from dubins_airplane import (BoxObstacle, DecentralizedPlanningError, ElevationMap,
                            FlightWorld, NetworkSettings, NoPathError, backend_name,
                            check_separation, plan_safe_rrt_star, simulate_decentralized)
from dubins_airplane.decentralized import holding_flight
from dubins_airplane.native import library_path
from dubins_airplane.traffic import plan_safe_rrt_star_traffic
from examples.example_fleet import make_fleet_scenario

VERSION = "AirLogistics-3D-v1"
FAMILIES = ("reference", "slope", "terrain", "gates", "combined")
AGENTS = {1: "C", 3: "BCD", 5: "ABCDE"}
PROFILES = {
    "ideal": dict(minimum_delay=0, maximum_delay=0, loss=0, duplicates=0),
    "normal": dict(minimum_delay=0, maximum_delay=.15, loss=.1, duplicates=.1),
    "slow": dict(minimum_delay=3, maximum_delay=3, loss=0, duplicates=.1),
}


def case_parameters(family, map_seed):
    if family not in FAMILIES or not isinstance(map_seed, int) or not 0 <= map_seed <= 9:
        raise ValueError("use a known family and map seed 0..9")
    if family == "reference" and map_seed != 0:
        raise ValueError("reference is the original fixed example: use map seed 0")
    if family == "reference":
        return {"family": family, "map_seed": map_seed}
    rng = Random(map_seed)
    ramp = [rng.uniform(200, 350), rng.uniform(5650, 5800)]
    peaks = [(x+rng.uniform(-180, 180), y+rng.uniform(-100, 100), a*rng.uniform(.8, 1.2), sx, sy)
             for x, y, a, sx, sy in ((2400, 550, 300, 600, 400),
                                     (3300, -600, 270, 650, 420),
                                     (4300, 450, 180, 550, 380))]
    gates = [(x+rng.uniform(-80, 80), center+rng.uniform(-80, 80), rng.uniform(160, 230))
             for x, center in ((1800, -250), (3500, 250), (4750, -250))]
    return {"family": family, "map_seed": map_seed, "ramp": ramp,
            "peaks": peaks if family in ("terrain", "combined") else [],
            "gates": gates if family in ("gates", "combined") else []}


def make_case(family="reference", map_seed=0, agents=5):
    """No planner-dependent rejection sampling: all published seeds are retained."""
    parameters = case_parameters(family, map_seed)
    if agents not in AGENTS:
        raise ValueError("agent count must be 1, 3 or 5")
    reference, requests = make_fleet_scenario("mountain_exchange_5")
    if family == "reference":
        world = reference
    else:
        def height(x, y):
            left, right = parameters["ramp"]
            s = max(0, min(1, (x-left)/(right-left)))
            relief = 0.0
            if family in ("terrain", "combined"):
                relief = sum(a*exp(-((x-cx)/sx)**2-((y-cy)/sy)**2)
                             for cx, cy, a, sx, sy in parameters["peaks"])
                relief -= 100*exp(-((x-3000)/800)**2-(y/240)**2)
            return max(0, min(1000, 1000*s + sin(pi*s)**2*relief))
        terrain = ElevationMap.from_function(-200, -1200, 10, 640, 240, height)
        boxes = []
        if family in ("gates", "combined"):
            for x, center, half_gap in parameters["gates"]:
                boxes.extend((BoxObstacle(x, -1200, 0, x+140, center-half_gap, 1600),
                              BoxObstacle(x, center+half_gap, 0, x+140, 1200, 1600)))
        world = FlightWorld(terrain, 40, 260, obstacles=boxes, obstacle_margin=5)
    return _select_case(world, requests, parameters, agents)


def _select_case(world, requests, parameters, agents):
    family, map_seed = parameters["family"], parameters["map_seed"]
    selected = tuple(r for r in requests if r.name in AGENTS[agents])
    if not all(world.validate_loiter(c) for r in selected for c in (r.start, r.goal)):
        raise ValueError("benchmark generator produced an unsafe endpoint circle")
    descriptor = {"suite": VERSION, "parameters": parameters, "agents": [asdict(r) for r in selected],
                  "map_size_m": [6400, 2400], "terrain_range_m": [0, 1000],
                  "cell_m": 10, "obstacles": [asdict(b) for b in world.obstacles],
                  "clearance_m": [40, 260], "obstacle_margin_m": 5,
                  "separation_m": 40, "horizon_s": 600, "planning_window_s": 15}
    # An ID of the published configuration, NOT a hash of floating-point terrain.
    encoded = json.dumps(descriptor, sort_keys=True, separators=(",", ":")).encode()
    descriptor["id"] = f"{family}-m{map_seed}-n{agents}-" + hashlib.sha256(encoded).hexdigest()[:12]
    return world, selected, descriptor


def quantile(values, fraction):
    """Linearly interpolated empirical quantile; no statistical bound implied."""
    ordered = sorted(values)
    if not ordered:
        return None
    position = fraction*(len(ordered)-1)
    index = int(position)
    return ordered[index] + (ordered[min(index+1, len(ordered)-1)]-ordered[index])*(position-index)


def timing_summary(calls):
    if not calls:
        return {"count": 0}
    times = [c["elapsed_ms"] for c in calls]
    return {"count": len(calls), "p50_ms": median(times), "p95_ms": quantile(times, .95),
            "p99_ms": quantile(times, .99), "max_ms": max(times),
            "success_fraction": sum(c["succeeded"] for c in calls)/len(calls),
            "successful_under_100ms_fraction": sum(c["succeeded"] and c["elapsed_ms"] < 100
                                                  for c in calls)/len(calls)}


def value_summary(values):
    if not values:
        return {"count": 0}
    return {"count": len(values), "min": min(values), "p50": median(values),
            "p95": quantile(values, .95), "max": max(values)}


def replay_local(world, requests, result, seed, iterations):
    """Replay identical local inputs sequentially, outside network/wall timing.

    Successful route history is only used to check replay fidelity. Neighbours
    are reconstructed from commits received BEFORE each original REPLAN event.
    This measures isolated search latency, not the cost of an online warm start.
    """
    roster = {r.name: r for r in requests}
    names = sorted(roster)
    committed = {f.request.name: f for f in result.plan.flights if f.duration > 0}
    first = {f.request.name: f for f in result.initial_flights}
    # An initial NoPath can be followed by a successful static retry. Such a
    # first usable route is in history, but not in first-attempt proposals.
    for snapshot in result.route_history:
        first.setdefault(snapshot.agent, snapshot.flight)
    snapshots = {(r.agent, r.attempt): r for r in result.route_history}
    holds = {n: holding_flight(r) for n, r in roster.items()}
    calls = []
    for call in result.planning_calls:
        r = roster[call.agent]
        kwargs = dict(iterations=iterations, seed=seed+names.index(r.name)+1009*(call.attempt-1))
        if call.mode == "traffic":
            retries = [e for e in result.events if e.agent == r.name and e.kind == "REPLAN"]
            began = retries[call.attempt-2].time
            received = {e.peer for e in result.events if e.agent == r.name and e.time <= began
                        and e.kind == "RECEIVE" and e.detail == "COMMIT"}
            peers = tuple(committed[n] if n in received else holds[n] for n in names if n != r.name)
            kwargs.update(start_pose=first[r.name].route.paths[0].start,
                          departure=first[r.name].departure, speed=r.speed, reservations=peers,
                          separation=40, horizon=600)
            planner = plan_safe_rrt_star_traffic
        else:
            kwargs["start_direction"] = "CCW"
            planner = plan_safe_rrt_star
        route = None
        started = perf_counter()
        try:
            route = planner(world, r.start, r.goal, r.radius, r.max_angle, **kwargs)
        except NoPathError:
            pass
        elapsed = 1000*(perf_counter()-started)
        same = (route is not None) == call.succeeded
        if route is not None and (r.name, call.attempt) in snapshots:
            same = same and isclose(route.length, snapshots[r.name, call.attempt].flight.route.length,
                                    rel_tol=1e-10, abs_tol=1e-5)
            same = same and route.paths == snapshots[r.name, call.attempt].flight.route.paths
        calls.append(dict(agent=r.name, attempt=call.attempt, mode=call.mode,
                          elapsed_ms=elapsed, succeeded=route is not None, matches_original=same,
                          roots=1 if call.mode == "traffic" else 8))
    return calls


def measure(world, requests, descriptor, profile, seed, iterations=600, replay=False):
    network = NetworkSettings(**PROFILES[profile], seed=17)
    failure = None
    try:
        result = simulate_decentralized(world, requests, network=network, seed=seed,
                                       iterations=iterations, planning_window=15)
    except DecentralizedPlanningError as error:
        result, failure = error.result, str(error)
        if result is None:
            raise
    # Diagnostics below are deliberately OUTSIDE the timed simulator call.
    audited = result.plan.validate(world)
    initial = result.initial_flights
    initial_conflicts = None
    if len(initial) == len(requests):
        initial_conflicts = [[a.request.name, b.request.name] for i, a in enumerate(initial)
                             for b in initial[i+1:] if not check_separation(a, b, 40, 600).safe]
    flights = result.plan.flights
    committed = {e.agent for e in result.events if e.kind == "COMMIT"}
    lower_bound = sum(hypot(max(0, hypot(r.goal.x-r.start.x, r.goal.y-r.start.y)
                                          -r.start.radius-r.goal.radius), r.goal.z-r.start.z)
                      for r in requests)
    full_length = sum(f.route.length for f in flights) if result.agreement_complete else None
    calls = [dict(asdict(c), roots=1 if c.mode == "traffic" else 8) for c in result.planning_calls]
    return {"case_id": descriptor["id"], "family": descriptor["parameters"]["family"],
            "map_seed": descriptor["parameters"]["map_seed"], "agents": len(requests),
            "network": profile, "seed": seed, "iterations": iterations,
            "agreement_complete": result.agreement_complete, "safety_valid": audited,
            "committed_agents": sorted(committed), "initial_conflict_pairs": initial_conflicts,
            "failure": failure, "wall_ms": result.wall_ms,
            "initial_stage_ms": result.plan.routing_ms, "negotiation_stage_ms": result.plan.scheduling_ms,
            "negotiation_sim_s": result.negotiation_time, "messages_sent": result.messages_sent,
            "messages_lost": result.messages_lost, "calls": calls,
            "isolated_calls": replay_local(world, requests, result, seed, iterations) if replay else [],
            "total_length_m": full_length,
            "length_over_euclidean_lower_bound": full_length/lower_bound if full_length is not None else None,
            "makespan_s": result.plan.makespan if result.agreement_complete else None,
            "per_agent": [{"name": f.request.name, "committed": f.request.name in committed,
                           "departure_s": f.departure if f.duration > 0 else None,
                           "arrival_s": f.arrival if f.duration > 0 else None,
                           "length_m": f.route.length if f.duration > 0 else None} for f in flights]}


def summarize(runs):
    """Group by exact instance AND network; do not pool unlike conditions."""
    groups = {}
    for run in runs:
        groups.setdefault(run["case_id"]+"/"+run["network"], []).append(run)
    summaries = {}
    for key, group in groups.items():
        complete = [r for r in group if r["agreement_complete"] and r["safety_valid"]]
        calls = [c for r in group for c in r["calls"]]
        isolated = [c for r in group for c in r["isolated_calls"]]
        summaries[key] = {"runs": len(group), "safe_complete_fraction": len(complete)/len(group),
                          "safety_failures": sum(not r["safety_valid"] for r in group),
                          "committed_agent_fraction": sum(len(r["committed_agents"]) for r in group)
                                                        /sum(r["agents"] for r in group),
                          "wall_p50_ms": median(r["wall_ms"] for r in group),
                          "wall_p95_ms": quantile([r["wall_ms"] for r in group], .95),
                          "total_length_m": value_summary([r["total_length_m"] for r in complete]),
                          "individual_length_m": value_summary([a["length_m"] for r in complete
                                                                 for a in r["per_agent"] if a["length_m"] is not None]),
                          "makespan_s": value_summary([r["makespan_s"] for r in complete]),
                          "repeat_searches": value_summary([sum(c["attempt"] > 1 for c in r["calls"]) for r in group]),
                          "system_static": timing_summary([c for c in calls if c["mode"] == "static"]),
                          "system_traffic": timing_summary([c for c in calls if c["mode"] == "traffic"]),
                          "isolated_static": timing_summary([c for c in isolated if c["mode"] == "static"]),
                          "isolated_traffic": timing_summary([c for c in isolated if c["mode"] == "traffic"])}
    return summaries


def positive(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--family", choices=FAMILIES, nargs="+", default=["reference", "combined"])
    parser.add_argument("--map-seeds", type=int, choices=range(10), nargs="+", default=[0])
    parser.add_argument("--agents", type=int, choices=AGENTS, nargs="+", default=[5])
    parser.add_argument("--network", choices=PROFILES, nargs="+", default=["normal"])
    parser.add_argument("--runs", type=positive, default=3)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--iterations", type=positive, default=600)
    parser.add_argument("--warmup", type=int, choices=(0, 1), default=1)
    parser.add_argument("--backend", choices=("cpp", "python"), default="cpp")
    parser.add_argument("--replay-local", action="store_true", help="also replay local searches sequentially")
    parser.add_argument("--manifest-only", action="store_true", help="generate/validate instances without planning")
    parser.add_argument("--output", type=Path, default=Path("outputs/benchmark_fleet/result.json"))
    args = parser.parse_args(argv)
    os.environ["DUBINS_BACKEND"] = args.backend
    backend = backend_name()  # Fail explicitly if the required C++ library is absent.
    root = Path(__file__).resolve().parents[1]
    sources = [Path(__file__).resolve(), root/"dubins_airplane/decentralized.py"]
    if backend == "cpp":
        sources.append(library_path())
    report = {"suite": VERSION, "created_utc": datetime.now(timezone.utc).isoformat(),
              "configuration": {**vars(args), "output": str(args.output)},
              "environment": {"backend": backend, "python": platform.python_version(),
                              "platform": platform.platform(), "processor": platform.processor(),
                              "logical_cpus": os.cpu_count(),
                              "sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}},
              "instances": [], "runs": [], "summary": {}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    journal = args.output.with_suffix(".runs.jsonl")
    if not args.manifest_only:
        journal.write_text(json.dumps({"type": "header", "suite": VERSION,
                           "configuration": report["configuration"], "environment": report["environment"]},
                          ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")
    for family in dict.fromkeys(args.family):
        for map_seed in dict.fromkeys(args.map_seeds):
            if family == "reference" and map_seed != 0:
                continue
            started = perf_counter()
            world, all_requests, base = make_case(family, map_seed, 5)
            setup_ms = 1000*(perf_counter()-started)
            for agents in dict.fromkeys(args.agents):
                _, requests, descriptor = _select_case(world, all_requests, base["parameters"], agents)
                descriptor["map_setup_ms"] = setup_ms  # One geometry shared by the agent-count variants.
                report["instances"].append(descriptor)
                if args.manifest_only:
                    print(f"Validated: {descriptor['id']}", flush=True)
                    continue
                with journal.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps({"type": "instance", "instance": descriptor},
                                            ensure_ascii=False, allow_nan=False)+"\n")
                for profile in dict.fromkeys(args.network):
                    if args.warmup:
                        measure(world, requests, descriptor, profile, args.seed-1, args.iterations)
                    for seed in range(args.seed, args.seed+args.runs):
                        run = measure(world, requests, descriptor, profile, seed, args.iterations,
                                      replay=args.replay_local)
                        report["runs"].append(run)
                        with journal.open("a", encoding="utf-8") as stream:
                            stream.write(json.dumps({"type": "run", "run": run},
                                                    ensure_ascii=False, allow_nan=False)+"\n")
                        print(f"{descriptor['id']} {profile} seed={seed}: "
                              f"complete={run['agreement_complete']} safe={run['safety_valid']} "
                              f"commits={len(run['committed_agents'])}/{agents} wall={run['wall_ms']:.1f} ms",
                              flush=True)
    if not report["instances"]:
        parser.error("no instances selected (reference is available only at map seed 0)")
    report["summary"] = summarize(report["runs"])
    args.output.write_text(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2), encoding="utf-8")
    for key, row in report["summary"].items():
        print(f"{key}: complete {row['safe_complete_fraction']:.0%}, "
              f"wall p50={row['wall_p50_ms']:.1f} ms")
        for metric in ("system_static", "system_traffic", "isolated_static", "isolated_traffic"):
            timing = row[metric]
            if timing["count"]:
                print(f"  {metric}: n={timing['count']} p50={timing['p50_ms']:.1f} "
                      f"p95={timing['p95_ms']:.1f} max={timing['max_ms']:.1f} ms "
                      f"successful+<100ms={timing['successful_under_100ms_fraction']:.0%}")
    print(f"Saved: {args.output}")
    bad = any(not r["safety_valid"] or any(not c["matches_original"] for c in r["isolated_calls"])
              for r in report["runs"])
    return 1 if bad else 0  # A reported safe timeout/no-path is NOT a checker failure.


if __name__ == "__main__":
    raise SystemExit(main())
