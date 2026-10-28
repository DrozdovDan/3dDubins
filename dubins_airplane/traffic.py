"""Time-aware directed RRT*: peer trajectories constrain tree edges, not just the final route.

Departure state/time are fixed. Arrival time is departure + cumulative 3D length / speed.
No waiting edges, speed changes or extra holding laps are used to resolve conflicts.
This remains a finite-budget prioritized planner, not complete/optimal joint planning.
"""

from dataclasses import dataclass
from math import atan2, cos, isfinite, pi, sin
from types import SimpleNamespace

from .core import Pose
from .rrt_star import _plan_safe_rrt_star_python, _validate_inputs


class _MovingEdge:
    def __init__(self, path, speed, departure):
        self.path, self.departure = path, departure
        self.request = SimpleNamespace(speed=speed)

    def pose_at(self, at):
        p = self.path
        d = min(p.length, max(0, (at-self.departure) * self.request.speed))
        return p.pose_at(min(p.horizontal_length, d*p.horizontal_length/p.length)) if p.length else p.start


class _CircleMotion:
    def __init__(self, circle, endpoint, speed, reference_time):
        from .fleet import _circle_direction
        self.circle, self.reference_time = circle, reference_time
        self.phase = atan2(endpoint.y-circle.y, endpoint.x-circle.x)
        self.direction = _circle_direction(circle, endpoint)
        self.request = SimpleNamespace(speed=speed)

    def pose_at(self, at):
        c = self.circle
        phase = self.phase + self.direction*self.request.speed*(at-self.reference_time)/c.radius
        return Pose(c.x+c.radius*cos(phase), c.y+c.radius*sin(phase), c.z, phase+self.direction*pi/2)


@dataclass(frozen=True)
class _TrafficConstraints:
    start: object
    goal: object
    speed: float
    departure: float
    reservations: tuple
    separation: float
    horizon: float

    def hold_free(self, root):
        from .fleet import _separation_python, loiter_disk_distance
        own = _CircleMotion(self.start, root, self.speed, self.departure)
        return all(loiter_disk_distance(self.goal, peer.request.goal) > self.separation+1e-4
                   and _separation_python(own, peer, self.separation, 0, self.departure, 2, .01).safe
                   for peer in self.reservations)

    def edge_free(self, edge, cost):
        from .fleet import _separation_python
        at = self.departure + cost/self.speed
        end = at + edge.length/self.speed
        if end > self.horizon:
            return False
        own = _MovingEdge(edge, self.speed, at)
        return all(_separation_python(own, peer, self.separation, at, end, 2, .01).safe
                   for peer in self.reservations)

    def terminal_free(self, edge, cost):
        from .fleet import _separation_python
        arrival = self.departure + (cost+edge.length)/self.speed
        own = _CircleMotion(self.goal, edge.goal, self.speed, arrival)
        # Separated terminal disks already certify motion after both arrivals.
        return all(min(self.horizon, peer.arrival) <= arrival or
                   _separation_python(own, peer, self.separation, arrival,
                                      min(self.horizon, peer.arrival), 2, .01).safe
                   for peer in self.reservations)


def plan_safe_rrt_star_traffic(world, start, goal, radius, max_angle, *, start_pose,
                               departure, speed, reservations, separation=40, horizon=600,
                               iterations=600, seed=7, circle_samples=8, goal_bias=.15,
                               neighbor_radius=400, collision_step=10, relative_gap=0.0):
    """Rebuild spatial geometry around received peer flights at a fixed departure.

    Peers retain their waiting, travel and terminal motion. Unknown peers must
    be passed as holding flights. Every parent edge and goal connection is
    continuously checked; rewiring rejects unsafe descendant retiming. A copied
    incumbent preserves the best complete safe route even if later tree changes
    invalidate an old goal connection. No asymptotic-optimality guarantee.
    """
    from .fleet import _on_circle, _circle_direction
    from .native import get_library, plan_rrt
    from .terrain import FlightWorld
    _validate_inputs(world, start, goal, radius, max_angle, iterations, circle_samples,
                     goal_bias, neighbor_radius, collision_step, None)
    if (not all(isfinite(v) for v in (departure, speed, separation, horizon))
            or departure < 0 or speed <= 0 or separation <= 0 or horizon <= departure
            or not _on_circle(start, start_pose) or _circle_direction(start, start_pose) != 1):
        raise ValueError("invalid fixed departure state/time or traffic limits")
    reservations = tuple(reservations)
    if any(peer.arrival > horizon for peer in reservations):
        raise ValueError("horizon must cover peer arrivals before terminal disks certify later motion")
    traffic = _TrafficConstraints(start, goal, speed, departure, reservations, separation, horizon)
    parameters = dict(iterations=iterations, seed=seed, circle_samples=circle_samples,
                      start_direction="CCW", goal_bias=goal_bias, neighbor_radius=neighbor_radius,
                      collision_step=collision_step, time_limit=None, on_first_solution=None,
                      start_pose=start_pose, traffic=traffic, relative_gap=relative_gap)
    lib = get_library()
    if lib is None or type(world) is not FlightWorld:
        return _plan_safe_rrt_star_python(world, start, goal, radius, max_angle, **parameters)
    return plan_rrt(lib, world, start, goal, radius, max_angle, **parameters)
