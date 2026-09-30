"""Directed Dubins RRT* between safe periodic sets (Lim et al., Section V).

The terrain/loiter construction follows Sections III--IV; steering reuses
our exact Owen planner. This readable Python version is not the authors'
ROS/OMPL implementation or their faster suboptimal medium-altitude metric.
Finite circle discretization and CSC-only steering limit completeness;
no asymptotic-optimality or real-flight safety claim is made for this code.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import hypot, isfinite, log, pi
from random import Random
from time import monotonic
from typing import Callable

from .core import DubinsAirplanePath, Pose, plan_dubins_airplane
from .terrain import FlightWorld, LoiterCircle


class NoPathError(RuntimeError):
    """No feasible route was found within the search budget, not a proof of impossibility."""


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


def plan_safe_rrt_star(
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
    """
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
    nodes = [_Node(pose) for pose in start.states(circle_samples, start_direction)]
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

    def free(edge):
        return world.path_is_free(edge, max_step=collision_step,
                                  min_step=min(0.05, collision_step))

    def connect_goal(node):
        choices = [edge for pose in goals if (edge := connect(node.pose, pose)) is not None]
        for edge in sorted(choices, key=lambda path: path.length):
            if free(edge):
                node.goal_edge = edge
                return

    for node in nodes:
        connect_goal(node)

    def best():
        candidates = [i for i, node in enumerate(nodes) if node.goal_edge is not None]
        return min(candidates, key=lambda i: nodes[i].cost + nodes[i].goal_edge.length) if candidates else None

    def record(iteration):
        index = best()
        if index is not None:
            cost = nodes[index].cost + nodes[index].goal_edge.length
            if not history or cost < history[-1][1] - 1e-8:
                if not history and on_first_solution is not None:
                    on_first_solution(iteration)
                history.append((iteration, cost))

    record(0)
    completed = 0
    for iteration in range(1, iterations + 1):
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
            if free(edge):
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
                    and node.cost + outward.length < other.cost - 1e-8 and free(outward)):
                _rewire(nodes, i, index, outward)
                rewires += 1
        connect_goal(node)
        record(iteration)

    chosen = best()
    if chosen is None:
        raise NoPathError(f"no route in {completed} iterations ({len(nodes)} nodes); "
                          "increase the budget, change seed/loiters, or inspect the map")
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
