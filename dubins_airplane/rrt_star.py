"""Directed Dubins RRT* between safe periodic sets (Lim et al., Section V).

The terrain/loiter construction follows Sections III--IV; steering reuses
our exact Owen planner. This readable Python version is not the authors'
ROS/OMPL implementation or their faster suboptimal medium-altitude metric.
Finite circle discretization and CSC-only steering limit completeness;
no asymptotic-optimality or real-flight safety claim is made for this code.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import hypot, isfinite, log, pi, sin
from random import Random
from time import monotonic
from typing import Callable

from .core import DubinsAirplanePath, Pose, _plan_dubins_airplane_python as plan_dubins_airplane
from .terrain import FlightWorld, LoiterCircle


class NoPathError(RuntimeError):
    """No feasible route was found within the search budget, not a proof of impossibility."""


def _validate_relative_gap(relative_gap):
    if not isfinite(relative_gap) or not 0 <= relative_gap <= 1:
        raise ValueError("relative_gap must be finite and in [0, 1]; zero disables early stopping")


def _route_lower_bound(roots, goals, radius, max_angle):
    """Physical bound for these discrete endpoints, also valid with obstacles/traffic."""
    def bound(a, b):
        distance = max(hypot(a.x-b.x, a.y-b.y, a.z-b.z), abs(a.z-b.z)/sin(max_angle))
        return max(0.0, distance - 1e-7*max(1.0, 2*radius) - 1e-8*max(1.0, distance))
    return min(bound(a, b) for a in roots for b in goals)


@dataclass(frozen=True)
class PlanningResult:
    paths: tuple[DubinsAirplanePath, ...]
    start_loiter: LoiterCircle
    goal_loiter: LoiterCircle
    iterations: int
    nodes: int
    rewires: int
    cost_history: tuple[tuple[int, float], ...]

    @property
    def length(self):
        return sum(path.length for path in self.paths)

    def sample(self, max_horizontal_step=5.0):
        points = []
        for path in self.paths:
            sampled = path.sample(max_horizontal_step)
            points.extend(sampled if not points else sampled[1:])
        return points


@dataclass
class _Node:
    pose: Pose
    cost: float = 0.0
    parent: int | None = None
    edge: DubinsAirplanePath | None = None
    children: set[int] = field(default_factory=set)
    goal_edge: DubinsAirplanePath | None = None


def _rewire(nodes: list[_Node], index: int, parent: int, edge: DubinsAirplanePath):
    """Replace an incoming edge and propagate its cost change to every descendant."""
    node = nodes[index]
    if node.parent is None:
        raise ValueError("start roots cannot be rewired")
    ancestor = parent
    while ancestor is not None:
        if ancestor == index:
            raise ValueError("rewiring would create a cycle")
        ancestor = nodes[ancestor].parent
    nodes[node.parent].children.remove(index)
    nodes[parent].children.add(index)
    difference = nodes[parent].cost + edge.length - node.cost
    node.parent, node.edge = parent, edge
    stack = [index]
    while stack:
        current = stack.pop()
        nodes[current].cost += difference
        stack.extend(nodes[current].children)


def _traffic_rewire_subtree(nodes, index, parent, edge, traffic):
    """Preview changed arrival times without mutating the tree; None rejects."""
    ancestor = parent
    while ancestor is not None:
        if ancestor == index:
            return None
        ancestor = nodes[ancestor].parent
    difference = nodes[parent].cost + edge.length - nodes[index].cost
    affected, queue = [], [index]
    while queue:
        current = queue.pop()
        affected.append(current)
        if (current != index and not traffic.edge_free(nodes[current].edge,
                nodes[current].cost - nodes[current].edge.length + difference)):
            return None
        queue.extend(nodes[current].children)
    return affected


def _validate_inputs(world, start, goal, radius, max_flight_path_angle, iterations,
                     circle_samples, goal_bias, neighbor_radius, collision_step, time_limit):
    if (not all(isfinite(v) for v in (radius, max_flight_path_angle, goal_bias,
                                    neighbor_radius, collision_step))
            or radius <= 0 or not 0 < max_flight_path_angle < pi / 2
            or not 0 <= goal_bias <= 1 or neighbor_radius <= 0 or collision_step <= 0
            or not isinstance(iterations, int) or iterations <= 0
            or not isinstance(circle_samples, int) or circle_samples < 4
            or (time_limit is not None and (not isfinite(time_limit) or time_limit <= 0))):
        raise ValueError("invalid RRT* limits, sample count or search budget")
    if start.radius < radius or goal.radius < radius:
        raise ValueError("loiter radii must be at least the aircraft minimum radius")
    if not world.validate_loiter(start) or not world.validate_loiter(goal):
        raise ValueError("both start and goal must be safe loiter disks")


def _plan_safe_rrt_star_python(
    world: FlightWorld,
    start: LoiterCircle,
    goal: LoiterCircle,
    radius: float,
    max_flight_path_angle: float,
    *,
    iterations: int = 600,
    seed: int = 7,
    circle_samples: int = 8,
    start_direction: str = "CCW",
    goal_bias: float = 0.15,
    neighbor_radius: float = 400.0,
    collision_step: float = 10.0,
    time_limit: float | None = None,
    on_first_solution: Callable[[int], None] | None = None,
    start_pose=None,
    traffic=None,
    relative_gap: float = 0.0,
) -> PlanningResult:
    """Find and improve a collision-free route between two validated loiters.

    All start phases are zero-cost roots (waiting on the start circle is
    excluded from the connection cost). Goal phases include both CW/CCW.
    Connections, nearest neighbours, parent choice and rewiring are directed:
    d_D(a,b) is not d_D(b,a). Full-state steering connects an entire sample;
    no straight-line XY shortcut is used as an aircraft trajectory.
    Seed fixes sampling; time_limit is a soft wall-clock budget checked
    between iterations (an individual connection check is not interrupted).
    on_first_solution is called once, immediately when a feasible route first
    appears (iteration 0 is possible). Its execution time is part of planning.
    relative_gap > 0 stops once a fully checked route is within that fraction
    of a physical length lower bound for the sampled endpoints. Otherwise the
    search keeps its full iteration cap. Zero preserves fixed-budget behavior.
    """
    _validate_inputs(world, start, goal, radius, max_flight_path_angle, iterations,
                     circle_samples, goal_bias, neighbor_radius, collision_step, time_limit)
    _validate_relative_gap(relative_gap)
    nodes = [_Node(pose) for pose in ([start_pose] if start_pose is not None else start.states(circle_samples, start_direction))]
    if traffic is not None and not traffic.hold_free(nodes[0].pose):
        raise NoPathError("fixed initial hold or terminal disks conflict with received reservations")
    roots = len(nodes)
    goals = goal.states(circle_samples, "CW") + goal.states(circle_samples, "CCW")
    rng = Random(seed)
    grid = world.terrain
    scale = 2 * hypot(grid.x_max - grid.x_min, grid.y_max - grid.y_min)
    deadline = None if time_limit is None else monotonic() + time_limit
    history = []
    rewires = 0

    def connect(a, b):
        try:
            return plan_dubins_airplane(a, b, radius, max_flight_path_angle)
        except ValueError:
            # Close-pose CSC/length equations can be unsupported by Owen.
            return None

    def free(edge, cost=0, terminal=False):
        return (world.path_is_free(edge, max_step=collision_step, min_step=min(0.05, collision_step))
                and (traffic is None or (traffic.edge_free(edge, cost)
                     and (not terminal or traffic.terminal_free(edge, cost)))))

    def connect_goal(node):
        node.goal_edge = None
        choices = [edge for pose in goals if (edge := connect(node.pose, pose)) is not None]
        for edge in sorted(choices, key=lambda path: path.length):
            if free(edge, node.cost, terminal=True):
                node.goal_edge = edge
                return

    for node in nodes:
        connect_goal(node)

    def best():
        candidates = [i for i, node in enumerate(nodes) if node.goal_edge is not None]
        return min(candidates, key=lambda i: nodes[i].cost + nodes[i].goal_edge.length) if candidates else None

    incumbent = []
    def record(iteration):
        nonlocal incumbent
        index = best()
        if index is not None:
            cost = nodes[index].cost + nodes[index].goal_edge.length
            if not history or cost < history[-1][1] - 1e-8:
                if not history and on_first_solution is not None:
                    on_first_solution(iteration)
                history.append((iteration, cost))
                if traffic is not None:
                    incumbent = [nodes[index].goal_edge]
                    current = index
                    while nodes[current].parent is not None:
                        incumbent.append(nodes[current].edge)
                        current = nodes[current].parent
                    incumbent.reverse()

    record(0)
    completed = 0
    route_bound = (_route_lower_bound([n.pose for n in nodes], goals, radius, max_flight_path_angle)
                   if relative_gap else 0)
    for iteration in range(1, iterations + 1):
        if relative_gap and history and history[-1][1] <= (1+relative_gap)*route_bound:
            break
        if deadline is not None and monotonic() >= deadline:
            break
        completed = iteration
        if rng.random() < goal_bias:
            sampled = rng.choice(goals)
        else:
            x, y = rng.uniform(grid.x_min, grid.x_max), rng.uniform(grid.y_min, grid.y_max)
            band = world.altitude_band(x, y)
            if band is None:
                continue
            sampled = Pose(x, y, rng.uniform(*band), rng.uniform(-pi, pi))
        if not world.is_free(sampled):
            continue

        # Exact directed incoming Dubins lengths, not Euclidean nearest.
        incoming = [(i, edge) for i, node in enumerate(nodes)
                    if (edge := connect(node.pose, sampled)) is not None and edge.length > 1e-8]
        if not incoming:
            continue
        nearest = min(incoming, key=lambda item: item[1].length)[0]
        n = len(nodes) + 1
        neighbourhood = min(neighbor_radius, scale * (log(n) / n)**0.25)  # state dimension 4
        parents = [(i, edge) for i, edge in incoming
                   if i == nearest or edge.length <= neighbourhood]
        chosen = None
        for i, edge in sorted(parents, key=lambda item: nodes[item[0]].cost + item[1].length):
            if free(edge, nodes[i].cost):
                chosen = (i, edge)
                break
        if chosen is None:
            continue
        parent, edge = chosen
        index = len(nodes)
        node = _Node(sampled, nodes[parent].cost + edge.length, parent, edge)
        nodes.append(node)
        nodes[parent].children.add(index)

        # An old goal connection remains valid after rewiring, but its cost
        # changes along with the entire subtree. Recompute best from costs.
        for i in range(roots, index):
            other = nodes[i]
            if hypot(other.pose.x - sampled.x, other.pose.y - sampled.y,
                     other.pose.z - sampled.z) > neighbourhood:
                continue
            outward = connect(sampled, other.pose)
            if (outward is not None and outward.length <= neighbourhood
                    and node.cost + outward.length < other.cost - 1e-8 and free(outward, node.cost)):
                affected = []
                if traffic is not None:
                    affected = _traffic_rewire_subtree(nodes, i, index, outward, traffic)
                    if affected is None:
                        continue
                _rewire(nodes, i, index, outward)
                for current in affected:
                    connect_goal(nodes[current])
                rewires += 1
        connect_goal(node)
        record(iteration)

    chosen = best()
    if chosen is None and not incumbent:
        raise NoPathError(f"no route in {completed} iterations ({len(nodes)} nodes); "
                          "increase the budget, change seed/loiters, or inspect the map")
    if traffic is not None and incumbent:
        paths = incumbent
    else:
        paths = [nodes[chosen].goal_edge]
        current = chosen
        while nodes[current].parent is not None:
            paths.append(nodes[current].edge)
            current = nodes[current].parent
        paths.reverse()
    # A goal-biased node may already equal a terminal state. Do not expose
    # its zero-length goal connector as a spurious extra route leg.
    nonzero = tuple(path for path in paths if path.length > 1e-8)
    return PlanningResult(nonzero or tuple(paths[:1]), start, goal, completed,
                          len(nodes), rewires, tuple(history))


def plan_safe_rrt_star(
    world: FlightWorld, start: LoiterCircle, goal: LoiterCircle,
    radius: float, max_flight_path_angle: float, *,
    iterations: int = 600, seed: int = 7, circle_samples: int = 8,
    start_direction: str = "CCW", goal_bias: float = 0.15,
    neighbor_radius: float = 400.0, collision_step: float = 10.0,
    time_limit: float | None = None,
    on_first_solution: Callable[[int], None] | None = None,
    relative_gap: float = 0.0,
) -> PlanningResult:
    from .native import get_library, plan_rrt
    lib = get_library()
    parameters = dict(iterations=iterations, seed=seed, circle_samples=circle_samples,
                      start_direction=start_direction, goal_bias=goal_bias,
                      neighbor_radius=neighbor_radius, collision_step=collision_step,
                      time_limit=time_limit, on_first_solution=on_first_solution,
                      relative_gap=relative_gap)
    if lib is None or type(world) is not FlightWorld:
        return _plan_safe_rrt_star_python(world, start, goal, radius, max_flight_path_angle, **parameters)
    return plan_rrt(lib, world, start, goal, radius, max_flight_path_angle, **parameters)


plan_safe_rrt_star.__doc__ = _plan_safe_rrt_star_python.__doc__
