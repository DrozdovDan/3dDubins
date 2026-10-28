"""Different multi-agent tasks, once each, with spatial replanning enabled.

python -m examples.benchmark_scenarios
python -m examples.benchmark_scenarios --plot-only outputs/benchmark_swarm_1km_adaptive/results.json
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from itertools import product
from math import ceil, exp, pi, radians, sin
import os
from pathlib import Path
import platform
from statistics import median
from time import perf_counter

from dubins_airplane import (AgentRequest, BoxObstacle, DecentralizedPlanningError,
                            ElevationMap, FlightWorld, NetworkSettings, backend_name,
                            check_separation, simulate_decentralized)
from dubins_airplane.fleet import loiter_disk_distance
from dubins_airplane.native import library_path

SUITE = "AirLogistics-GroupMissions-v5-1km"
FLIGHT_BOUNDS = dict(x=[0, 1000], y=[-500, 500], z=[150, 1150])
BOUNDARY_TOLERANCE = 1e-6
TOPOLOGIES = {
    "open": "Открытый склон",
    "pinch": "Один проход",
    "fork": "Два прохода",
    "slalom": "Слалом",
    "islands": "Отдельные препятствия",
    "mountains": "Горный транзит",
}
SCENARIOS = {"transfer": "Перелёт группы", "gather": "Сбор группы", "reconfigure": "Перестроение группы"}
DIRECTIONS = {"ascent": "Подъём роя", "descent": "Спуск роя"}
LEGACY_SCENARIOS = {"exchange": "Встречный обмен", "crossing": "Перекрёстный обмен", "convoy": "Попутное пересечение"}
GOAL_REGION = dict(x=[730, 970], y=[-240, 240], z=[1150, 1150])
COUNTS = (2, 4, 5, 8)
# One fixed random stream for reproducibility, NOT an axis of the benchmark.
PLANNER_SEED = 7
NETWORK = dict(minimum_delay=0, maximum_delay=.15, loss=.1, duplicates=.1, seed=17)
IDEAL_NETWORK = dict(NETWORK, maximum_delay=0, loss=0, duplicates=0)


def network_description(configuration):
    settings = configuration.get('network')
    if settings is None:
        return ''  # Older/synthetic reports may omit transport metadata.
    if all(settings.get(k, 0) == 0 for k in ('minimum_delay', 'maximum_delay', 'loss', 'duplicates')):
        return 'Мгновенный обмен: 0 мс, без потерь и дубликатов'
    return (f"Обмен: {settings['minimum_delay']*1000:g}–{settings['maximum_delay']*1000:g} мс; "
            f"потери {settings['loss']*100:g}%; дубликаты {settings['duplicates']*100:g}%")


def goal_region(direction="ascent"):
    """ONE destination region for the whole swarm, on its destination terrace."""
    if direction not in DIRECTIONS:
        raise ValueError("use ascent or descent for the whole swarm")
    return dict(x=list(GOAL_REGION['x']) if direction == 'ascent' else [30, 270],
                y=list(GOAL_REGION['y']), z=[1150, 1150] if direction == 'ascent' else [150, 150])


def make_scene(topology, scenario):
    """Explicit geometries, not random maps or rejection-sampled successful cases."""
    if topology not in TOPOLOGIES or scenario not in SCENARIOS:
        raise ValueError("unknown topology or scenario")
    variant = tuple(SCENARIOS).index(scenario)
    sign = -1 if variant == 1 else 1
    shift = (0, 0, 15)[variant]

    def height(x, y):
        s = max(0, min(1, (x-300)/400))
        relief = 0
        if topology == "mountains":
            yy = sign*y
            relief = (140*exp(-((x-420-shift)/100)**2-((yy-190)/150)**2)
                      + 130*exp(-((x-540)/110)**2-((yy+190)/160)**2)
                      + 90*exp(-((x-640+shift)/90)**2-((yy-170)/140)**2)
                      - 60*exp(-((x-500)/150)**2-(yy/100)**2))
        return max(0, min(1000, 1000*s + sin(pi*s)**2*relief))

    boxes = []

    def wall(x, gaps):
        cursor = -500
        for low, high in sorted(gaps):
            boxes.append(BoxObstacle(x, cursor, 150, x+25, low, 1150))
            cursor = high
        boxes.append(BoxObstacle(x, cursor, 150, x+25, 500, 1150))

    if topology == "pinch":
        center = (0, -80, 80)[variant]
        half_width = (100, 115, 130)[variant]
        wall(480+shift, [(center-half_width, center+half_width)])
    elif topology == "fork":
        half_width = (100, 115, 130)[variant]
        wall(480+shift, [(-230-half_width, -230+half_width), (230-half_width, 230+half_width)])
    elif topology == "slalom":
        half_width = (150, 165, 180)[variant]
        for x, center in ((350, -140), (480, 140), (610, -140)):
            center = sign*center
            wall(x+shift, [(center-half_width, center+half_width)])
    elif topology == "islands":
        for x, center in ((350, -200), (430, 200), (510, -180), (590, 180), (650, 0)):
            center = sign*center
            boxes.append(BoxObstacle(x+shift, center-65, 150, x+35+shift, center+65, 1150))
    elif topology == "mountains":
        for x, low, high in ((350, -500, -30), (480, 30, 500), (610, -500, -30)):
            a, b = sorted((sign*low, sign*high))
            boxes.append(BoxObstacle(x, a, 150, x+25, b, 1150))
    terrain = ElevationMap.from_function(0, -500, 10, 100, 100, height)
    world = FlightWorld(terrain, 40, 260, obstacles=boxes, obstacle_margin=5)
    # Intersect Lim's terrain band with the actual 1000 m vertical workspace.
    # The tiny tolerance permits exact boundary loiters under strict free-space tests.
    world.lower = tuple(tuple(max(FLIGHT_BOUNDS['z'][0]-BOUNDARY_TOLERANCE, z) for z in row) for row in world.lower)
    world.upper = tuple(tuple(min(FLIGHT_BOUNDS['z'][1]+BOUNDARY_TOLERANCE, z) for z in row) for row in world.upper)
    return world


def make_requests(world, scenario, agents, direction="ascent"):
    """Nested rosters: N=2/4/5/8 selects the first N of the SAME eight tasks."""
    if scenario not in SCENARIOS or agents not in range(1, 9) or direction not in DIRECTIONS:
        raise ValueError("use a known scenario, ascent/descent and 1..8 agents")
    # One team moves to ONE common destination region. Prefixes keep the
    # same tasks when the fleet grows, with 130+ m between loiter disk centres.
    compact = [(80, -65), (80, 65), (220, -65), (220, 65),
               (80, -195), (80, 195), (220, -195), (220, 195)]
    dispersed = [(80, -360), (80, 360), (240, -120), (240, 120),
                 (80, -120), (80, 120), (240, -360), (240, 360)]
    starts = dispersed if scenario == "gather" else compact
    goals = [(x+700, -y if scenario == "reconfigure" else y) for x, y in compact]
    if direction == 'descent':
        # Reflect endpoint layouts, NOT the map or precomputed paths. Gathering
        # stays gathering: do not merely reverse an ascent into a dispersal.
        starts = [(1000-x, y) for x, y in starts]
        goals = [(1000-x, y) for x, y in goals]
    start_z, goal_z = (150, 1150) if direction == 'ascent' else (1150, 150)
    region = goal_region(direction)
    coordinates = list(zip(starts, goals))
    requests = tuple(AgentRequest(chr(65+i), world.choose_loiter(*start, 40, altitude=start_z),
                                  world.choose_loiter(*goal, 40, altitude=goal_z),
                                  speed=20, radius=40, max_angle=radians(15),
                                  earliest_departure=25, initial_phase=-10)
                     for i, (start, goal) in enumerate(coordinates[:agents]))
    if not all(world.validate_loiter(c) for r in requests for c in (r.start, r.goal)):
        raise ValueError("unsafe initial/terminal circle")
    if not all(region["x"][0] <= r.goal.x-r.goal.radius <= r.goal.x+r.goal.radius <= region["x"][1]
               and region["y"][0] <= r.goal.y-r.goal.radius <= r.goal.y+r.goal.radius <= region["y"][1]
               and r.goal.z == region["z"][0] for r in requests):
        raise ValueError("terminal disks must fit inside the common mission goal region")
    for i, request in enumerate(requests):
        for other in requests[:i]:
            if min(loiter_disk_distance(request.start, other.start),
                   loiter_disk_distance(request.goal, other.goal)) <= 40:
                raise ValueError("endpoint disks are not separated")
    return requests


def describe_scene(world, topology, scenario, direction="ascent"):
    grid = world.terrain
    return dict(id=f"{topology}/{scenario}/{direction}", topology=topology, scenario=scenario,
                extent_m=[grid.x_min, grid.x_max, grid.y_min, grid.y_max],
                flight_bounds_m=FLIGHT_BOUNDS,
                cell_m=10, terrain_range_m=[0, 1000],
                obstacles=[asdict(b) for b in world.obstacles],
                mission=dict(kind=scenario, direction=direction, objective="all_agents_reach_common_region",
                             goal_region_m=goal_region(direction), formation_required=False, speed_mps=20),
                # Only the overview is decimated; planning always uses the full 10 m grid.
                overview_heights=[list(row[::5]) for row in grid.heights[::5]], overview_cell_m=50)


def flight_record(flight):
    return dict(agent=flight.request.name, departure_s=flight.departure,
                arrival_s=flight.arrival, length_m=flight.route.length,
                rrt_iterations=flight.route.iterations, rrt_nodes=flight.route.nodes,
                points=[[round(p.x, 4), round(p.y, 4), round(p.z, 4)] for p in flight.route.sample(10)])


def measure_task(world, requests, topology, scenario, iterations=600, planning_window=100,
                 max_route_attempts=8, retry_iterations=3000, direction="ascent", relative_gap=0.0,
                 network=None):
    """The original decentralized spatial replanner, never a fixed-route scheduler."""
    if direction not in DIRECTIONS or any(r.goal.z-r.start.z != (1000 if direction == 'ascent' else -1000)
                                         for r in requests):
        raise ValueError('all agents must have the declared swarm mission direction')
    failure = None
    try:
        result = simulate_decentralized(world, requests, iterations=iterations, seed=PLANNER_SEED,
                                       planning_window=planning_window, max_route_attempts=max_route_attempts,
                                       retry_iterations=retry_iterations,
                                       relative_gap=relative_gap,
                                       network=network if network is not None else NetworkSettings(**NETWORK))
    except DecentralizedPlanningError as error:
        result, failure = error.result, str(error)
        if result is None:
            raise
    # Everything below, including route export and the extra audit, is OUTSIDE wall_ms.
    safe = result.plan.validate(world)
    complete = result.agreement_complete and safe
    initial = {f.request.name: f for f in result.initial_flights}
    final = {f.request.name: f for f in result.plan.flights if f.duration > 0}
    pairs = None
    if len(initial) == len(requests):
        flights = list(initial.values())
        pairs = [[a.request.name, b.request.name] for i, a in enumerate(flights) for b in flights[i+1:]
                 if not check_separation(a, b, 40, 600).safe]
    changed = [n for n, f in final.items() if n in initial and f.route.paths != initial[n].route.paths]
    delays = {n: f.departure-initial[n].departure for n, f in final.items() if n in initial}
    calls = [asdict(c) for c in result.planning_calls]
    return dict(id=f"{topology}/{scenario}/{direction}/n{len(requests)}", scene_id=f"{topology}/{scenario}/{direction}",
                topology=topology, scenario=scenario, direction=direction, agents=len(requests),
                complete=complete, safety_valid=safe, failure=failure,
                committed_agents=sorted(final), wall_ms=result.wall_ms,
                wall_per_agent_ms=result.wall_ms/len(requests),
                initial_stage_ms=result.plan.routing_ms, coordination_stage_ms=result.plan.scheduling_ms,
                total_length_m=sum(f.route.length for f in final.values()) if complete else None,
                mean_length_m=sum(f.route.length for f in final.values())/len(requests) if complete else None,
                makespan_s=result.plan.makespan if complete else None,
                initial_conflicts=pairs, geometry_changed_agents=sorted(changed),
                extra_departure_delay_s=delays,
                replan_calls=sum(c["mode"] == "traffic" for c in calls), calls=calls,
                rrt_calls=len(calls), iterations_per_call=iterations, max_route_attempts=max_route_attempts,
                retry_iterations=retry_iterations,
                relative_gap=relative_gap,
                total_iteration_budget=sum(c['iteration_budget'] for c in calls),
                requests=[asdict(r) for r in requests],
                initial_routes=[flight_record(f) for f in initial.values()],
                final_routes=[flight_record(f) for f in final.values()],
                route_history=[dict(time_s=s.time, attempt=s.attempt, **flight_record(s.flight))
                               for s in result.route_history],
                events=[asdict(e) for e in result.events if e.kind in
                        ("CONFLICT", "REPLAN", "COMMIT", "FAILED", "TIMEOUT")])


def planning_time_metrics(run):
    """Amortised group wall time and measured local duration per ALL RRT* calls."""
    calls = run.get('calls')
    count = run.get('rrt_calls', len(calls) if calls is not None else 0)
    wall = run['wall_ms']/count if count else None
    local = sum(c['elapsed_ms'] for c in calls)/len(calls) if calls else None
    return wall, local


def summary_rows(runs):
    """Aggregate DIFFERENT workloads by fleet size, never repeated random seeds."""
    rows = []
    for count in sorted({r["agents"] for r in runs}):
        selected = [r for r in runs if r["agents"] == count]
        good = [r for r in selected if r["complete"] and r["safety_valid"]]
        timings = [planning_time_metrics(r) for r in selected]
        wall_per_call = [w for w, _ in timings if w is not None]
        local_per_call = [t for _, t in timings if t is not None]
        planning_counts = [r['rrt_calls']/count for r in selected if r.get('rrt_calls') is not None]
        rows.append(dict(agents=count, tasks=len(selected), complete=len(good),
                         wall_median_ms=median(r["wall_ms"] for r in selected),
                         wall_per_agent_median_ms=median(r["wall_ms"]/r["agents"] for r in selected),
                         length_median_m=median(r["total_length_m"] for r in good) if good else None,
                         mean_agent_length_m=median(r["total_length_m"]/r["agents"] for r in good) if good else None,
                         replan_calls=sum(r["replan_calls"] for r in selected),
                         replans_per_agent_median=median(r["replan_calls"]/count for r in selected),
                         replans_per_agent_mean=sum(r["replan_calls"]/count for r in selected)/len(selected),
                         plannings_per_agent_median=median(planning_counts) if planning_counts else None,
                         plannings_per_agent_mean=sum(planning_counts)/len(planning_counts) if planning_counts else None,
                         wall_per_planning_median_ms=median(wall_per_call) if wall_per_call else None,
                         local_per_planning_median_ms=median(local_per_call) if local_per_call else None,
                         wall_per_planning_tasks=len(wall_per_call), local_per_planning_tasks=len(local_per_call),
                         changed_agents=sum(len(r["geometry_changed_agents"]) for r in selected)))
    return rows


def write_metric_figures(report, destination):
    """Pool ascent/descent observations by N, without rerunning or rendering routes."""
    if not report.get("runs") or any(r.get("scenario") not in SCENARIOS for r in report["runs"]):
        raise ValueError("expected shared group missions (transfer/gather/reconfigure); old independent-flight measurements cannot be relabelled as swarm missions")
    from examples.plot_benchmark_fleet import draw_boxes
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    destination.mkdir(parents=True, exist_ok=True)
    runs = report["runs"]
    incomplete = any(not r['complete'] for r in runs)
    counts = sorted({r["agents"] for r in runs})
    grouped = [[r for r in runs if r['agents'] == n] for n in counts]
    labels = [f"{n} {'агент' if n == 1 else 'агента' if n < 5 else 'агентов'}\n"
              f"{sum(r['complete'] and r['safety_valid'] for r in group)}/{len(group)} задач"
              for n, group in zip(counts, grouped)]
    for field, divisor, name, title, ylabel, color in (
            ("wall_ms", 1, "time", "Полное время миссии на агента", "T / N, мс на агента", "#80b5db"),
            ("total_length_m", 1000, "length", "Длина маршрутов, нормированная на число агентов", "ΣLᵢ / N, км на агента", "#8bbfa6"),
            ("rrt_calls", 1, "plannings", "Все вызовы планировщика на агента", "P / N, вызовов на агента", "#d8af78"),
            ("replan_calls", 1, "replans", "Количество перепланирований на агента", "R / N, вызовов на агента", "#d8af78")):
        fig = plt.figure(figsize=(16, 9), facecolor='white')
        axis = fig.add_axes((.09, .25, .85, .50))
        values = [[r[field]/r["agents"]/divisor for r in group if r[field] is not None
                   and (field != "total_length_m" or (r["complete"] and r["safety_valid"]))] for group in grouped]
        # All observations are already scattered below: do not draw outliers a
        # second time at the box centre, which would visually duplicate tasks.
        draw_boxes(axis, values, labels, color, show_fliers=False)
        for position, group in enumerate(grouped, 1):
            matching = [r for r in group if r[field] is not None
                        and (field != "total_length_m" or (r["complete"] and r["safety_valid"]))]
            for i, r in enumerate(matching):
                axis.scatter(position-.13+.26*i/max(1, len(matching)-1), r[field]/r["agents"]/divisor,
                             color="#31596f" if r["complete"] else "#be3c43",
                             marker="o" if r["complete"] else "x", s=28, alpha=.6, zorder=3)
        if field != "total_length_m":
            axis.set_ylim(bottom=0)
        axis.set_ylabel(ylabel, fontsize=14, color='#516576', labelpad=12)
        axis.tick_params(labelsize=13)
        fig.text(.055, .935, title, fontsize=25, weight='bold', color='#142637')
        fig.text(.055, .885, f'{len(runs)} миссий · одна точка — одна миссия · подъём и спуск объединены',
                 fontsize=14, color='#516576')
        transport_note = network_description(report.get('configuration', {}))
        if transport_note:
            fig.text(.055, .842, transport_note, fontsize=12, color='#516576')
        medians = '  ·  '.join(f'{n}: {median(v):.2f}'.replace('.', ',') if v else f'{n}: —'
                              for n, v in zip(counts, values))
        fig.text(.09, .13, f'Медианы   {medians}', fontsize=12, color='#142637')
        fig.text(.055, .081, 'Линия — медиана; ящик — средние 50%; усы — 1,5 IQR. Показаны все миссии.',
                 fontsize=11.5, color='#516576')
        note = {"wall_ms": "T — весь цикл группы: первоначальные пути, согласование, повторные поиски и итоговая проверка.",
                "total_length_m": "Средняя пространственная длина на агента; полные безопасные планы всех агентов.",
                "rrt_calls": "P — все вызовы RRT*: первоначальные, повторные статические и с учётом соседей, включая неудачные.",
                "replan_calls": "R — все вызовы RRT* с учётом соседей, включая неудачные. Первоначальный поиск не входит."}[field]
        note += (" Красный × — неполная миссия." if incomplete else " Все миссии спланированы полностью.")
        fig.text(.055, .035, note, fontsize=10.5, color='#516576')
        fig.savefig(destination/f"{name}.png", dpi=240)
        fig.savefig(destination/f"{name}.svg")
        plt.close(fig)

    # Redraw from observations in one slide-native layout, not a raster collage.
    from matplotlib.ticker import FuncFormatter

    fig = plt.figure(figsize=(16, 9), facecolor='white')
    ink, muted = '#142637', '#516576'
    fig.text(.055, .935, 'Роевая маршрутизация: метрики',
             fontsize=27, weight='bold', color=ink)
    topologies = len({r.get('topology') for r in runs})
    scenarios = len({r['scenario'] for r in runs})
    directions = len({r.get('direction', 'ascent') for r in runs})
    fig.text(.055, .885, f'{len(runs)} миссий · {topologies} топологий · {scenarios} задания · '
             f'{directions} направления · подъём и спуск объединены', fontsize=14, color=muted)
    transport_note = network_description(report.get('configuration', {}))
    if transport_note:
        fig.text(.055, .842, transport_note, fontsize=12, color=muted)
    for left, field, divisor, title, ylabel, color, unit in (
            (.065, 'wall_ms', 1, 'Полное время миссии', 'T/N, мс на агента', '#80b5db', 'мс'),
            (.395, 'total_length_m', 1000, 'Длина маршрута', 'ΣLᵢ/N, км на агента', '#8bbfa6', 'км'),
            (.725, 'rrt_calls', 1, 'Все планирования', 'P/N, вызовов на агента', '#d8af78', 'выз.')):
        axis = fig.add_axes((left, .285, .25, .485))
        values = [[r[field]/r['agents']/divisor for r in group if r[field] is not None
                   and (field != 'total_length_m' or (r['complete'] and r['safety_valid']))] for group in grouped]
        draw_boxes(axis, values, [str(n) for n in counts], color, show_fliers=False)
        for position, group in enumerate(grouped, 1):
            matching = [r for r in group if r[field] is not None
                        and (field != 'total_length_m' or (r['complete'] and r['safety_valid']))]
            for i, run in enumerate(matching):
                axis.scatter(position-.14+.28*i/max(1, len(matching)-1),
                             run[field]/run['agents']/divisor,
                             color='#31596f' if run['complete'] else '#be3c43',
                             marker='o' if run['complete'] else 'x', s=20, alpha=.55, zorder=3)
        axis.set_title(title, loc='left', fontsize=18, weight='bold', color=ink, pad=22)
        axis.set_ylabel(ylabel, fontsize=12, labelpad=10, color=muted)
        axis.set_xlabel('Число агентов', fontsize=14, labelpad=10, color=muted)
        axis.tick_params(labelsize=13)
        axis.yaxis.set_major_formatter(FuncFormatter(lambda value, _: f'{value:g}'.replace('.', ',')))
        axis.grid(axis='y', color='#e5ebf0', linewidth=.8)
        axis.set_axisbelow(True)
        if field != 'total_length_m':
            axis.set_ylim(bottom=0)
        medians = '  ·  '.join(f"{n}: {median(v):.2f}".replace('.', ',') if v else f'{n}: —'
                              for n, v in zip(counts, values))
        fig.text(left, .185, f'Медианы ({unit}/агент)', fontsize=11, color=muted)
        fig.text(left, .15, medians, fontsize=10.5, color=ink)
    sizes = sorted({len(group) for group in grouped})
    sample_note = (f'По {sizes[0]} миссий на каждый размер роя.' if len(sizes) == 1 else
                   'Число миссий зависит от размера роя.')
    good = sum(r['complete'] and r['safety_valid'] for r in runs)
    fig.text(.055, .103, f'{sample_note} Полных безопасных планов: {good}/{len(runs)}. '
             'Точка — одна миссия; линия — медиана.', fontsize=12, color=muted)
    fig.text(.055, .067, 'Ящик — средние 50%; усы — 1,5 IQR. Показаны все наблюдения, включая выбросы.',
             fontsize=12, color=muted)
    fig.text(.055, .031, 'T — весь цикл группы с проверкой. P — все вызовы RRT*, включая первоначальные и неудачные. '
             'Длительность отдельных вызовов показана отдельно.', fontsize=10.5, color=muted)
    fig.savefig(destination/'metrics.png', dpi=240)
    fig.savefig(destination/'metrics.svg')
    plt.close(fig)
    write_planning_time_scatter(report, destination)
    write_planning_time_scatter(report, destination, count_field='replan_calls')
    write_time_per_planning(report, destination)
    write_planning_call_times(report, destination)


def write_planning_time_scatter(report, destination, *, count_field='rrt_calls'):
    """All local RRT* calls (initial, static retries, traffic), including failures."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.ticker import AutoMinorLocator, FuncFormatter, MaxNLocator

    destination.mkdir(parents=True, exist_ok=True)
    runs = report['runs']
    counts = sorted({r['agents'] for r in runs})
    colors = ('#2878b5', '#d98627', '#24917c', '#8662ad', '#bc4c65', '#72843a', '#568a9d', '#8d6c52')
    ink, muted = '#142637', '#516576'
    all_calls = count_field == 'rrt_calls'
    title = 'Количество планирований и полное время' if all_calls else 'Перепланирования и полное время'
    noun_count, symbol = ('Планирования', 'P') if all_calls else ('Перепланирования', 'R')
    stem = 'planning_vs_time' if all_calls else 'replans_vs_time'
    fig = plt.figure(figsize=(16, 9), facecolor='white')
    fig.text(.055, .935, title, fontsize=26, weight='bold', color=ink)
    fig.text(.055, .885, f'{len(runs)} миссий · одна точка — одна миссия · подъём и спуск объединены',
             fontsize=14, color=muted)
    note = network_description(report.get('configuration', {}))
    if note:
        fig.text(.055, .842, note, fontsize=12, color=muted)
    axis = fig.add_axes((.085, .22, .73, .55))
    for i, count in enumerate(counts):
        group = [r for r in runs if r['agents'] == count]
        first = True
        for safe in (True, False):
            selected = [r for r in group if bool(r['complete'] and r['safety_valid']) == safe]
            if not selected:
                continue
            noun = 'агент' if count == 1 else 'агента' if count < 5 else 'агентов'
            axis.scatter([r[count_field]/count for r in selected],
                         [r['wall_ms']/count for r in selected],
                         s=75, color=colors[i % len(colors)], alpha=.75,
                         marker='o' if safe else 'X', edgecolors='white', linewidths=.5,
                         label=f'{count} {noun}' if first else '_nolegend_', zorder=3)
            first = False
    axis.set_xlabel(f'{noun_count} на агента, {symbol}/N', fontsize=15, labelpad=13, color=muted)
    axis.set_ylabel('Полное время на агента, T/N, мс', fontsize=15, labelpad=13, color=muted)
    axis.set_xlim(left=max(0, axis.get_xlim()[0]))
    axis.set_ylim(bottom=0)
    axis.tick_params(labelsize=13)
    formatter = FuncFormatter(lambda value, _: f'{value:g}'.replace('.', ','))
    axis.xaxis.set_major_formatter(formatter)
    # Keep a detailed linear scale without crowding larger archived timings.
    # The current 0–900 ms data get 50 ms labels and 10 ms minor ticks.
    axis.yaxis.set_major_locator(MaxNLocator(nbins=20, steps=[1, 2, 2.5, 5, 10]))
    axis.yaxis.set_minor_locator(AutoMinorLocator(5))
    axis.yaxis.set_major_formatter(formatter)
    axis.tick_params(axis='y', which='major', labelsize=11)
    axis.grid(color='#e5ebf0', linewidth=.8)
    axis.tick_params(axis='y', which='minor', length=3, color='#a5b3bf')
    axis.set_axisbelow(True)
    axis.spines[['top', 'right']].set_visible(False)
    axis.legend(title='Размер роя', loc='upper left', bbox_to_anchor=(1.025, 1),
                frameon=False, fontsize=13, title_fontsize=14, labelspacing=1.1)
    fig.text(.055, .097, 'T — весь цикл группы с согласованием, перепланированием и проверкой. '
             f'{symbol} — ' + ('все вызовы RRT*, включая неудачные.' if all_calls
                               else 'вызовы RRT* с учётом соседей, включая неудачные.'), fontsize=11, color=muted)
    footer = ('Начальный поиск ' + ('включён.' if all_calls else 'не входит в R.') +
              ' Одна точка — одна миссия. Координаты показаны без смещения; точки могут перекрываться.')
    if any(not r['complete'] or not r['safety_valid'] for r in runs):
        footer += ' Маркер × — неполная или небезопасная миссия.'
    fig.text(.055, .052, footer, fontsize=10.5, color=muted)
    fig.savefig(destination/f'{stem}.png', dpi=240)
    fig.savefig(destination/f'{stem}.svg')
    plt.close(fig)


def write_time_per_planning(report, destination):
    """Ratios are formed within each mission before comparing fleet sizes."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.ticker import FuncFormatter, MultipleLocator
    from examples.plot_benchmark_fleet import draw_boxes

    destination.mkdir(parents=True, exist_ok=True)
    runs = report['runs']
    counts = sorted({r['agents'] for r in runs})
    grouped = [[r for r in runs if r['agents'] == n] for n in counts]
    timings = [[planning_time_metrics(r) for r in group] for group in grouped]
    values = [[[pair[index] for pair in group if pair[index] is not None]
               for group in timings] for index in (0, 1)]
    observed = [value for panel in values for group in panel for value in group]
    if not observed:
        return  # Archived reports without call counts/timers cannot supply this metric.
    upper = max(100, 100 * ceil(max(observed)*1.08/100))
    fig = plt.figure(figsize=(16, 9), facecolor='white')
    ink, muted = '#142637', '#516576'
    fig.text(.055, .935, 'Среднее время на вызов в каждой миссии', fontsize=26, weight='bold', color=ink)
    fig.text(.055, .885, f'{len(runs)} миссий · первоначальные поиски, повторные попытки и перепланирования',
             fontsize=14, color=muted)
    note = network_description(report.get('configuration', {}))
    if note:
        fig.text(.055, .842, note, fontsize=12, color=muted)
    for index, left, title, ylabel, color in (
            (0, .075, 'Полное время / число вызовов', 'T/P, мс на вызов', '#80b5db'),
            (1, .565, 'Средняя длительность вызовов RRT*', 'Σtᵢ/P, мс на вызов', '#8bbfa6')):
        axis = fig.add_axes((left, .285, .385, .485))
        draw_boxes(axis, values[index], [str(n) for n in counts], color, show_fliers=False)
        for position, group in enumerate(values[index], 1):
            axis.scatter([position-.14+.28*i/max(1, len(group)-1) for i in range(len(group))],
                         group, s=20, alpha=.6, color='#31596f', zorder=3)
        axis.set_title(title, loc='left', fontsize=18, weight='bold', color=ink, pad=22)
        axis.set_ylabel(ylabel, fontsize=14, color=muted, labelpad=10)
        axis.set_xlabel('Число агентов', fontsize=14, color=muted, labelpad=10)
        axis.set_ylim(0, upper)
        axis.tick_params(labelsize=13)
        axis.yaxis.set_major_locator(MultipleLocator(100))
        axis.yaxis.set_minor_locator(MultipleLocator(25))
        axis.yaxis.set_major_formatter(FuncFormatter(lambda value, _: f'{value:g}'.replace('.', ',')))
        axis.grid(axis='y', color='#e5ebf0', linewidth=.8)
        axis.grid(axis='y', which='minor', color='#f0f3f6', linewidth=.5)
        axis.set_axisbelow(True)
        medians = '  ·  '.join(f'{n}: {median(v):.1f}'.replace('.', ',') if v else f'{n}: —'
                              for n, v in zip(counts, values[index]))
        fig.text(left, .168, f'Медианы (мс/вызов)   {medians}', fontsize=11.5, color=ink)
    fig.text(.055, .103, '(T/N) / (P/N) = T/P. Одна точка — одна миссия; линия — медиана, ящик — средние 50%, усы — 1,5 IQR.',
             fontsize=11.5, color=muted)
    fig.text(.055, .067, 'P — все вызовы RRT*, включая неудачные. tᵢ — измеренная длительность отдельного вызова.',
             fontsize=11.5, color=muted)
    fig.text(.055, .031, 'Начальные поиски идут параллельно: T/P показывает удельное время системы, Σtᵢ/P — среднюю длительность локального поиска.',
             fontsize=11, color=muted)
    fig.savefig(destination/'time_per_planning.png', dpi=240)
    fig.savefig(destination/'time_per_planning.svg')
    plt.close(fig)


def planning_call_groups(runs):
    """Separate measured local calls by their cap and role; never pool budgets."""
    groups = {}
    for run in runs:
        for call in run.get('calls', []):
            budget = call['iteration_budget']
            kind = ('initial' if call['attempt'] == 1 else
                    'traffic' if call['mode'] == 'traffic' else 'static_retry')
            groups.setdefault((budget, kind), []).append(call)
    return groups


def planning_call_summary(runs):
    groups = planning_call_groups(runs)
    total = sum(c['elapsed_ms'] for group in groups.values() for c in group)
    order = {'initial': 0, 'static_retry': 1, 'traffic': 2}
    return [dict(iteration_budget=budget, kind=kind, count=len(group),
                 failed=sum(not c['succeeded'] for c in group),
                 median_ms=median(c['elapsed_ms'] for c in group),
                 sum_ms=sum(c['elapsed_ms'] for c in group),
                 share_of_call_time=sum(c['elapsed_ms'] for c in group)/total if total else 0)
            for (budget, kind), group in sorted(groups.items(), key=lambda item: (item[0][0], order[item[0][1]]))]


def write_planning_call_times(report, destination):
    """Actual individual-call durations, with a separate panel for each cap."""
    groups = planning_call_groups(report['runs'])
    if not groups:
        return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MultipleLocator
    from examples.plot_benchmark_fleet import draw_boxes

    budgets = sorted({budget for budget, _ in groups})
    names = {'initial': 'Первый поиск', 'static_retry': 'Статический повтор', 'traffic': 'С соседями'}
    total_calls = sum(len(group) for group in groups.values())
    total_ms = sum(c['elapsed_ms'] for group in groups.values() for c in group)
    fig = plt.figure(figsize=(16, 9), facecolor='white')
    ink, muted = '#142637', '#516576'
    fig.text(.055, .935, 'Длительность отдельных вызовов RRT*', fontsize=26, weight='bold', color=ink)
    fig.text(.055, .885, f'{total_calls} вызовов в {len(report["runs"])} миссиях · одна точка — один вызов',
             fontsize=14, color=muted)
    note = network_description(report.get('configuration', {}))
    if note:
        fig.text(.055, .842, note, fontsize=12, color=muted)
    for index, budget in enumerate(budgets):
        kinds = [kind for kind in names if (budget, kind) in groups]
        selected = [groups[budget, kind] for kind in kinds]
        datasets = [[c['elapsed_ms'] for c in group] for group in selected]
        left = .075+index*.49 if len(budgets) == 2 else .075+index*.9/len(budgets)
        width = .385 if len(budgets) == 2 else .85/len(budgets)
        axis = fig.add_axes((left, .285, width, .435))
        labels = [f'{names[kind]}\nn = {len(group)}' for kind, group in zip(kinds, selected)]
        draw_boxes(axis, datasets, labels, '#80b5db' if index == 0 else '#d8af78', show_fliers=False)
        for position, group in enumerate(selected, 1):
            for i, call in enumerate(group):
                axis.scatter(position-.18+.36*i/max(1, len(group)-1), call['elapsed_ms'],
                             color='#31596f' if call['succeeded'] else '#b84642',
                             marker='o' if call['succeeded'] else 'x',
                             s=10 if call['succeeded'] else 22, alpha=.4 if call['succeeded'] else .75,
                             zorder=3)
        maximum = max(max(v) for v in datasets)
        tick = 50 if maximum < 250 else 100 if maximum < 1000 else 500 if maximum < 5000 else 1000
        axis.set_ylim(0, max(tick, tick*ceil(maximum*1.08/tick)))
        axis.yaxis.set_major_locator(MultipleLocator(tick))
        axis.yaxis.set_minor_locator(MultipleLocator(tick/4))
        axis.grid(axis='y', which='minor', color='#f0f3f6', linewidth=.5)
        axis.tick_params(axis='y', labelsize=12)
        axis.set_ylabel('Длительность одного вызова, мс', fontsize=12, color=muted, labelpad=10)
        axis.set_title(f'Лимит {budget} итераций', loc='left', fontsize=20, weight='bold', color=ink, pad=40)
        cap_ms = sum(sum(v) for v in datasets)
        cap_count = sum(len(v) for v in datasets)
        share = 100*cap_ms/total_ms if total_ms else 0
        fig.text(left, .747, f'{cap_count} вызовов · {share:.1f}% суммы длительностей'.replace('.', ','),
                 fontsize=11.5, color=muted)
        medians = ' · '.join(f'{names[k]}: {median(v):.1f}'.replace('.', ',') for k, v in zip(kinds, datasets))
        fig.text(left, .168, f'Медианы, мс   {medians}', fontsize=10.5, color=ink)
    fig.text(.055, .103, 'Ящики показывают отдельные вызовы всех агентов. Линия — медиана; ящик — средние 50%; усы — 1,5 IQR.',
             fontsize=11, color=muted)
    fig.text(.055, .067, '× — путь не найден в данном вызове; его время учтено. Лимит — верхняя граница, возможна ранняя остановка.',
             fontsize=11, color=muted)
    fig.text(.055, .031, 'Доли рассчитаны по сумме длительностей вызовов; начальные поиски могут перекрываться во времени. Шкалы панелей различаются.',
             fontsize=10.5, color=muted)
    fig.savefig(destination/'planning_call_times.png', dpi=240)
    fig.savefig(destination/'planning_call_times.svg')
    plt.close(fig)


def write_report(report, output, *, render_route_views=True):
    destination = output.parent/"report"
    write_metric_figures(report, destination)
    runs = report["runs"]
    scenes = list(report["scenes"].values())
    extent = scenes[0]["extent_m"]
    map_km = ((extent[1]-extent[0])/1000, (extent[3]-extent[2])/1000)
    flight_bounds = scenes[0].get("flight_bounds_m")
    volume_label = (f"Область {map_km[0]:g} × {map_km[1]:g} × {(flight_bounds['z'][1]-flight_bounds['z'][0])/1000:g} км"
                    if flight_bounds else f"Карта {map_km[0]:g} × {map_km[1]:g} км")
    summaries = summary_rows(runs)
    (destination/'summary.json').write_text(json.dumps(summaries, ensure_ascii=False, indent=2), encoding='utf-8')
    call_summary = planning_call_summary(runs)
    (destination/'call_summary.json').write_text(json.dumps(call_summary, ensure_ascii=False, indent=2), encoding='utf-8')
    directions = [d for d in DIRECTIONS if any(r.get('direction', 'ascent') == d for r in runs)]
    from examples.render_benchmark_routes import render_routes
    if render_route_views:
        render_routes(report, destination)

    def number(value, divisor=1):
        return f"{value/divisor:.2f}" if value is not None else "—"

    conflicted_tasks = sum(bool(r["initial_conflicts"]) for r in runs)
    initial_complete = sum(r["initial_conflicts"] is not None for r in runs)
    gap = report['configuration'].get('relative_gap', 0)
    quality_note = (f"Адаптивный поиск: ранняя остановка при длине не более чем на {100*gap:g}% выше "
                    "физической нижней оценки для дискретных начальных/конечных состояний. "
                    "Если оценка недостижима или путь ещё не найден, используется весь лимит. "
                    "Это локальный критерий длины, не гарантия оптимальности совместной миссии."
                    if gap else "Фиксированный бюджет: ранняя остановка по длине отключена.")
    completed_calls = [c for r in runs for c in r.get('calls', [])
                       if c.get('succeeded') and c.get('iterations') is not None]
    if completed_calls:
        early = sum(c['iterations'] < c['iteration_budget'] for c in completed_calls)
        quality_note += (f" Успешных локальных поисков: {len(completed_calls)}; "
                         f"остановились раньше лимита: {early}. Медиана фактических итераций "
                         f"успешного поиска: {median(c['iterations'] for c in completed_calls):g}. "
                         "Неудачные вызовы сохранены отдельно, их время включено в T.")
    lines = ["# Совместные миссии группы в объёме 1 × 1 × 1 км", "",
             network_description(report['configuration']), "",
             f"Задач: {len(runs)}. Каждая выполнена **один раз**. Нет серии из 30 сидов и изолированного воспроизведения поисков.", "",
             f"Решено: **{sum(r['complete'] for r in runs)}/{len(runs)}**. "
             f"Нарушений проверки безопасности: **{sum(not r['safety_valid'] for r in runs)}**. "
             f"Исходные конфликты обнаружены в **{conflicted_tasks}/{initial_complete}** задачах, "
             "для которых построены все первоначальные пути; в остальных этот показатель неизвестен. "
             f"Пространственных перепланирований: **{sum(r['replan_calls'] for r in runs)}**; "
             f"путей с изменённой геометрией: **{sum(len(r['geometry_changed_agents']) for r in runs)}**.", "",
             "Одна задача = топология + сценарий движения + направление ВСЕГО роя + число агентов. При увеличении числа агентов добавляются задания "
             "из одного фиксированного списка; прежние задания не меняются.", "",
             "Все агенты выполняют одну общую миссию: движение на одной скорости 20 м/с к одному району назначения "
             "с раздельными безопасными конечными кругами. Успех группы — завершены задания ВСЕХ агентов. "
             "По согласованной постановке жёсткий строй и верхняя граница расстояния между агентами не требуются.", "",
             "**Подъём роя:** все агенты летят 150 → 1150 м к общему верхнему району. "
             "**Спуск роя:** все агенты летят 1150 → 150 м к общему нижнему району. "
             "Ни в одной миссии рой не разделён на встречные потоки. Спуск рассчитывается заново на той же карте, "
             "а не воспроизводится задом наперёд. Сбор остаётся сбором и при спуске.", "",
             "* **Перелёт группы** — компактная группа перемещается к общей цели, сохраняя взаимное расположение конечных кругов.",
             "* **Сбор группы** — рассредоточенные агенты сходятся в компактный общий район.",
             "* **Перестроение группы** — компактная группа меняет расположение агентов при общем перелёте; целевые позиции зеркальны по y.", "",
             "## Какой алгоритм здесь работает", "",
             "Тот же `simulate_decentralized`: локальные C++ Owen/RRT*, обмен маршрутами, "
             "при конфликте новый пространственный путь с временными ограничениями от полученных соседей. "
             "Это не запуск статического поиска без координации и не разрешение конфликтов одним ожиданием. "
             f"Старые деревья не переиспользуются. Обычный лимит: {report['configuration']['iterations']} итераций; "
             f"после ненайденного пути: {report['configuration'].get('retry_iterations', report['configuration']['iterations'])} итераций; "
             "радиус 40 м, угол 15°, разделение >40 м.", "",
             "Перепланирование включено для КАЖДОЙ задачи, но вызывается по необходимости: если конфликтов нет, "
             "искусственно менять безопасные пути не нужно. В таблице видны исходные конфликты, вызовы "
             "перепланирования и самолёты с реально изменённой геометрией. В JSON сохранены исходные, "
             "промежуточные и конечные траектории и события REPLAN.", "",
             quality_note, "",
             "## Набор задач", "",
             f"{volume_label}; рельеф 0–1000 м; изменение высоты каждого рейса ровно 1 км; расчётная клетка 10 × 10 м. "
             + (f"Границы полёта: x={flight_bounds['x']}, y={flight_bounds['y']}, z={flight_bounds['z']} м. "
                "Высотный отсчёт сохранён прежним: слой 150–1150 м имеет толщину ровно 1000 м. "
                "Рельеф и запрещённые объёмы ограничивают движение внутри него; выход за куб запрещён. " if flight_bounds else "")
             + "Запрещённые объёмы перекрывают весь доступный высотный слой: перелететь их сверху нельзя. "
             "Для трёх групповых заданий заданы отдельные конкретные положения препятствий. "
             "Все геометрии заданы заранее, не отбирались по успешности планировщика.", "",
             "## Результаты по числу агентов", "",
             "Время — весь цикл на ноутбуке до результата и проверки, без подготовки карты, экспорта и графиков. "
             "Длина — сумма пространственных маршрутов всех агентов задачи, только у полных безопасных задач. "
             "В основных таблицах и на графиках значения нормированы: **T/N**, **ΣLᵢ/N**, **P/N** и **R/N**. "
             "Сначала делим результат каждой задачи на её N, затем считаем медиану. "
             "T/N — удельное время всей системы, **не** средняя задержка одного RRT*: начальные вызовы параллельны. "
             "Неполные задачи входят во время, перепланирования и успешность, но не в среднюю длину.", "",
             "**R/N** — число вызовов time-aware RRT* с учётом соседей в миссии, делённое на число агентов. "
             "Все такие вызовы учитываются, включая неудачные; первоначальные поиски и повторные статические "
             "поиски после NoPath сюда не входят. Сначала вычисляем R/N для каждой миссии, затем медиану "
             "и среднее по миссиям. Это не число агентов, поменявших путь: один агент может перестраиваться несколько раз.", "",
             f"**RRT*: до {report['configuration']['iterations']} итераций на обычный вызов; "
             f"{report['configuration'].get('retry_iterations', report['configuration']['iterations'])} после исчерпания поиска**, "
             f"не более {report['configuration'].get('max_route_attempts', 3)} локальных вызовов на агента. "
             f"Окно согласования {report['configuration'].get('planning_window_s', 15)} с симуляционного времени. "
             "Это не общий бюджет на группу и не повтор всей миссии. Время ВСЕХ поисков включено в T; "
             "в JSON calls.iteration_budget — лимит каждого вызова, calls.iterations — фактическое число "
             "итераций успешного поиска (null при неудаче), total_iteration_budget — сумма лимитов.", "",
             "| Агентов | Решено миссий | T/N, мс/агент (медиана) | ΣLᵢ/N, км/агент (медиана) | P/N (медиана) | P/N (среднее) | R/N (медиана) | R/N (среднее) | Пространственных перепланирований | Изменённых путей |",
             "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for row in summaries:
        lines.append(f"| {row['agents']} | {row['complete']}/{row['tasks']} | {number(row['wall_per_agent_median_ms'])} "
                     f"| {number(row['mean_agent_length_m'], 1000)} "
                     f"| {number(row['plannings_per_agent_median'])} | {number(row['plannings_per_agent_mean'])} "
                     f"| {number(row['replans_per_agent_median'])} | {number(row['replans_per_agent_mean'])} "
                     f"| {row['replan_calls']} | {row['changed_agents']} |")
    lines.extend(['', 'Подъём и спуск объединены: один ящик и одна строка сводки на каждое N. '
                  'В полном наборе это 36 миссий на N: 6 топологий × 3 задания × 2 направления. '
                  'Медиана считается по всем отдельным миссиям, не как среднее двух медиан направлений.'])
    if call_summary:
        call_names = {'initial': 'Первый поиск', 'static_retry': 'Статический повтор', 'traffic': 'С соседями'}
        lines.extend(['', '## Отдельные вызовы: тип и бюджет', '',
                      'Одна точка на этом рисунке — один локальный вызов, измеренный таймером вокруг планировщика. '
                      'Бюджеты представлены в разных панелях; каждый включает успешные и неудачные вызовы. '
                      'Лимит может быть исчерпан не полностью из-за адаптивной остановки.', '',
                      '| Тип | Лимит итераций | Вызовов | Путь не найден | Длительность, мс (медиана) | Доля суммы длительностей |',
                      '|---|---:|---:|---:|---:|---:|'])
        for row in call_summary:
            lines.append(f"| {call_names[row['kind']]} | {row['iteration_budget']} | {row['count']} | {row['failed']} "
                         f"| {number(row['median_ms'])} | {100*row['share_of_call_time']:.1f}% |")
        lines.extend(['', '![Отдельные вызовы: лимиты и типы](planning_call_times.png)', '',
                      '[PNG 16:9](planning_call_times.png) · [SVG](planning_call_times.svg) · [Данные JSON](call_summary.json)', ''])
    if any(row['wall_per_planning_tasks'] or row['local_per_planning_tasks'] for row in summaries):
        lines.extend(['', '## Время на одно планирование', '',
                      '**(T/N) / (P/N) = T/P** — удельное время всего цикла на один вызов. '
                      '**Σtᵢ/P** — средняя измеренная длительность локального вызова RRT*. '
                      'Включены все первоначальные поиски, повторные статические поиски и перепланирования, '
                      'в том числе неудачные. Начальные вызовы выполняются параллельно, поэтому суммы '
                      'их длительностей и реальное время группы различаются. Сначала считаем отношение '
                      'для каждой миссии, затем медиану по миссиям данного N.', '',
                      '| Агентов | Миссий с T/P | T/P, мс/вызов (медиана) | Миссий с Σtᵢ/P | Σtᵢ/P, мс/вызов (медиана) |',
                      '|---:|---:|---:|---:|---:|'])
        for row in summaries:
            lines.append(f"| {row['agents']} | {row['wall_per_planning_tasks']} | {number(row['wall_per_planning_median_ms'])} "
                         f"| {row['local_per_planning_tasks']} | {number(row['local_per_planning_median_ms'])} |")
        lines.extend(['', '![Время на одно планирование](time_per_planning.png)', '',
                      '[PNG для слайда 16:9](time_per_planning.png) · [Векторный SVG](time_per_planning.svg)', ''])
    lines.extend(["", "Ящики показывают разброс **между разными задачами**, не стабильность повторного запуска одной карты. "
                  "Линия — медиана; ящик — средние 50%; усы — по правилу 1,5 IQR; крайние точки сохранены. "
                  "Каждая задача также показана отдельной точкой.", "",
                  "![Полное время, длина и все планирования на агента](metrics.png)", "",
                  "[Для слайда 16:9, PNG 3840×2160](metrics.png) · [Векторный SVG](metrics.svg)", "",
                  "[Полное время](time.png) · [Длина](length.png) · [Все планирования](plannings.png) · [Перепланирования с соседями](replans.png)", "",
                  "[Нормированная сводка JSON](summary.json)", "",
                  "![Количество планирований и время на агента](planning_vs_time.png)", "",
                  "[Scatter plot, PNG 16:9](planning_vs_time.png) · [Векторный SVG](planning_vs_time.svg)", "",
                  "Каждая точка — одна миссия: X = P/N, Y = T/N в миллисекундах; цвет — число агентов. "
                  "P учитывает все вызовы RRT*: первоначальные, повторные статические поиски и перепланирования "
                  "с учётом соседей, включая неудачные вызовы. "
                  "Координаты не смещены, совпадающие точки могут перекрываться.", "",
                  "![Топологии и пути](routes.png)", "",
                  "[Картинка для слайда 16:9: PNG 3840×2160](routes.png) · [Снимки и сцены Rerun](rerun/)", "",
                  *[f"[{DIRECTIONS[d]}: все шесть топологий](routes_{d}.png)" for d in directions], '',
                  "## Каждая задача отдельно", "",
                  "«Начальные конфликты» — число конфликтующих пар исходных маршрутов; «—» значит не все исходные пути построены. "
                  "«Перепланирования» — вызовы time-aware RRT* с соседями, а не число независимых запусков бенчмарка. "
                  "Неполная задача остаётся в отчёте и не выдаётся за короткий успешный маршрут.", "",
                  "| Топология | Миссия | Направление всего роя | Агентов | Решено | T/N, мс/агент | ΣLᵢ/N, км/агент | R/N | Начальные конфликты | Перепланирования | Геометрия изменена |",
                  "|---|---|---|---:|---|---:|---:|---:|---:|---:|---|"])
    for run in runs:
        scenario_name = SCENARIOS.get(run['scenario'], LEGACY_SCENARIOS.get(run['scenario'], run['scenario']))
        lines.append(f"| {TOPOLOGIES[run['topology']]} | {scenario_name} | {DIRECTIONS[run.get('direction', 'ascent')]} | {run['agents']} "
                     f"| {'да' if run['complete'] else 'НЕТ'} | {number(run['wall_ms']/run['agents'])} "
                     f"| {number(run['total_length_m']/run['agents'] if run['total_length_m'] is not None else None, 1000)} "
                     f"| {number(run['replan_calls']/run['agents'])} "
                     f"| {len(run['initial_conflicts']) if run['initial_conflicts'] is not None else '—'} "
                     f"| {run['replan_calls']} | {','.join(run['geometry_changed_agents']) or '—'} |")
    lines.extend(["", "## Ограничения", "",
                  "Это собственный набор, не внешний стандарт и не сравнение с известным оптимумом. "
                  "Один запуск задачи не даёт оценки статистической надёжности на этой задаче. "
                  "Ненайденный план — результат ограниченного бюджета, не доказанная невозможность. "
                  "При неполном согласовании подтверждённые пути сохраняются, остальные агенты продолжают стартовые круги.", "",
                  "Использованы текущие приоритеты A→H. " + network_description(report['configuration']) + ". Это согласование перед вылетом "
                  "со стартовых кругов, не произвольное онлайн-изменение уже подтверждённой траектории в полёте.", "",
                  "Это согласование индивидуальных траекторий для общей миссии группы, не joint-state RRT* "
                  "и не доказанно оптимальная совместная минимизация суммы длин. Ограничения на растяжение группы в полёте не задано.", "",
                  "Картинка routes — настоящие снимки Rerun, вид сверху-сбоку: угол 55°, азимут −135°. "
                  "Рельеф отображён клетками 10×10 м; вертикальный масштаб не преувеличен. "
                  "Показаны только согласованные пути, без мешающего наложения первоначальных. "
                  "Безопасность подтверждается непрерывной 3D-проверкой, а не отсутствием пересечений на картинке.", "",
                  f"[Исходный JSON](../{output.name})", ""])
    (destination/"README.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"Report: {destination/'README.md'}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topology", nargs="+", choices=TOPOLOGIES, default=list(TOPOLOGIES))
    parser.add_argument("--scenario", nargs="+", choices=SCENARIOS, default=list(SCENARIOS))
    parser.add_argument("--direction", nargs="+", choices=DIRECTIONS, default=list(DIRECTIONS),
                        help="ascent/descent applies to the WHOLE swarm in each mission")
    parser.add_argument("--agents", nargs="+", type=int, choices=range(1, 9), default=list(COUNTS))
    parser.add_argument("--iterations", type=int, default=600)
    parser.add_argument("--retry-iterations", type=int, default=3000)
    parser.add_argument("--relative-gap", type=float, default=.03,
                        help="early stop within this fraction of the physical length bound; 0 = fixed budget")
    parser.add_argument("--network", choices=('normal', 'ideal'), default='normal',
                        help="normal: 0–150 ms, 10%% loss/duplicates; ideal: instant delivery, no loss/duplicates")
    parser.add_argument("--planning-window", type=float, default=100)
    parser.add_argument("--max-route-attempts", type=int, default=8,
                        help="local search-call budget per agent, including spatial replanning")
    parser.add_argument("--output", type=Path,
                        help="defaults to a separate normal/ideal benchmark directory")
    parser.add_argument("--plot-only", type=Path, help="redraw an existing results file without planning")
    parser.add_argument("--metrics-only", action="store_true",
                        help="with --plot-only: update charts/tables without rerendering Rerun route views")
    parser.add_argument("--no-plot", action="store_true", help="only measure; no Matplotlib needed")
    args = parser.parse_args(argv)
    if args.metrics_only and not args.plot_only:
        parser.error('--metrics-only requires --plot-only')
    if args.plot_only or not args.no_plot:
        try:
            import matplotlib  # Check BEFORE measuring, not after a long run.
            if not args.metrics_only:
                import rerun
            from PIL import Image
        except ImportError:
            parser.error("for figures: pip install -r requirements.txt -r rerun_scripts/requirements.txt; or measure with --no-plot")
    if args.plot_only:
        try:
            write_report(json.loads(args.plot_only.read_text(encoding="utf-8")), args.plot_only,
                         render_route_views=not args.metrics_only)
        except ValueError as error:
            parser.error(str(error))
        return 0
    if args.output is None:
        folder = 'benchmark_swarm_1km_instant' if args.network == 'ideal' else 'benchmark_swarm_1km_adaptive'
        args.output = Path('outputs')/folder/'results.json'
    if (args.iterations <= 0 or args.retry_iterations < args.iterations or args.max_route_attempts <= 0
            or not 0 < args.planning_window < 120 or not 0 <= args.relative_gap <= 1):
        parser.error("positive search budgets, planning window below max_delay=120 and relative gap in [0, 1] required")
    os.environ["DUBINS_BACKEND"] = "cpp"
    backend = backend_name()
    network = dict(IDEAL_NETWORK if args.network == 'ideal' else NETWORK)
    network_settings = NetworkSettings(**network)
    sources = [Path(__file__), Path(__file__).resolve().parents[1]/"dubins_airplane/decentralized.py", library_path()]
    report = dict(suite=SUITE, created_utc=datetime.now(timezone.utc).isoformat(),
                  configuration=dict(topologies=list(dict.fromkeys(args.topology)), scenarios=list(dict.fromkeys(args.scenario)),
                                     directions=list(dict.fromkeys(args.direction)),
                                     agents=sorted(set(args.agents)), iterations=args.iterations,
                                     retry_iterations=args.retry_iterations,
                                     relative_gap=args.relative_gap,
                                     planning_window_s=args.planning_window, max_route_attempts=args.max_route_attempts,
                                     planner_seed=PLANNER_SEED,
                                     spatial_replanning=True, network=network, network_profile=args.network,
                                     flight_bounds_m=FLIGHT_BOUNDS,
                                     shared_goal_regions_m={d: goal_region(d) for d in args.direction}, formation_required=False,
                                     metric_normalisation="wall_ms/N and total_length_m/N"),
                  environment=dict(backend=backend, python=platform.python_version(), platform=platform.platform(),
                                   processor=platform.processor(), sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}),
                  scenes={}, runs=[])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    journal = args.output.with_suffix(".runs.jsonl")
    journal.write_text(json.dumps({"type": "header", "configuration": report["configuration"],
                                   "environment": report["environment"]}, ensure_ascii=False)+"\n", encoding="utf-8")
    total = (len(report["configuration"]["topologies"])*len(report["configuration"]["scenarios"])
             *len(report["configuration"]["directions"])*len(report["configuration"]["agents"]))
    for topology in report["configuration"]["topologies"]:
        for scenario, direction in product(report["configuration"]["scenarios"], report["configuration"]["directions"]):
            began = perf_counter()
            world = make_scene(topology, scenario)
            scene = describe_scene(world, topology, scenario, direction)
            scene["setup_ms"] = (perf_counter()-began)*1000
            report["scenes"][scene["id"]] = scene
            with journal.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps({"type": "scene", "scene": scene}, ensure_ascii=False, allow_nan=False)+"\n")
            for count in report["configuration"]["agents"]:
                requests = make_requests(world, scenario, count, direction)
                run = measure_task(world, requests, topology, scenario, args.iterations, args.planning_window,
                                   args.max_route_attempts, args.retry_iterations, direction=direction,
                                   relative_gap=args.relative_gap, network=network_settings)
                # The limit remains an upper cap; successful searches may stop earlier.
                report["runs"].append(run)
                with journal.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps({"type": "run", "run": run}, ensure_ascii=False, allow_nan=False)+"\n")
                print(f"[{len(report['runs'])}/{total}] {run['id']}: complete={run['complete']} "
                      f"safe={run['safety_valid']} wall={run['wall_ms']:.1f} ms "
                      f"replans={run['replan_calls']} changed={','.join(run['geometry_changed_agents']) or '-'}", flush=True)
    report["summary"] = summary_rows(report["runs"])
    report["summary_by_direction"] = {d: summary_rows([r for r in report['runs'] if r.get('direction', 'ascent') == d])
                                       for d in report['configuration']['directions']}
    args.output.write_text(json.dumps(report, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    print(f"Saved: {args.output}", flush=True)
    if not args.no_plot:
        write_report(report, args.output)
    # A safe holding circle is NOT a completed mission or a successful benchmark.
    return int(any(not r["complete"] or not r["safety_valid"] for r in report["runs"]))


if __name__ == "__main__":
    raise SystemExit(main())
