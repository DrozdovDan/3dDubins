"""Centralized prioritized schedules for airborne Dubins airplanes.

Existing Owen/RRT* paths are retained. Conflicts are resolved by whole laps
on safe start loiters, not by hovering, teleporting, changing speed or
silently dropping agents after arrival. This is not joint-state RRT* or an
optimal/complete multi-agent planner. Safety is geometric, in a static world
with exact constant ground speeds and no tracking error/wind model.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, ceil, cos, floor, hypot, isfinite, pi, sin, tau
from time import perf_counter
from typing import Sequence

from .core import Pose
from .rrt_star import PlanningResult, plan_safe_rrt_star
from .terrain import FlightWorld, LoiterCircle


class NoScheduleError(RuntimeError):
    """No schedule for these fixed routes/order within the delay budget."""


@dataclass(frozen=True)
class AgentRequest:
    name: str
    start: LoiterCircle
    goal: LoiterCircle
    speed: float = 20.0  # Constant 3D ground speed in m/s, including loiters.
    radius: float = 40.0
    max_angle: float = pi / 12
    earliest_departure: float = 0.0
    initial_phase: float | None = None  # Measured polar angle on the CCW start circle at t=0.

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name or any(ch in self.name for ch in "/\\"):
            raise ValueError("agent name must be nonempty and contain no path separators")
        if (not all(isfinite(v) for v in (self.speed, self.radius, self.max_angle, self.earliest_departure))
                or self.speed <= 0 or self.radius <= 0 or not 0 < self.max_angle < pi / 2
                or self.earliest_departure < 0):
            raise ValueError("invalid speed, radius, angle or earliest departure")
        if self.initial_phase is not None and not isfinite(self.initial_phase):
            raise ValueError("initial circle phase must be finite")
        if min(self.start.radius, self.goal.radius) < self.radius:
            raise ValueError("both loiter radii must be at least the minimum turn radius")


def _circle_direction(circle, pose):
    phase = atan2(pose.y - circle.y, pose.x - circle.x)
    error = (pose.yaw - phase - pi / 2 + pi) % tau - pi
    return 1 if abs(error) < pi / 2 else -1


def _on_circle(circle, pose):
    phase = atan2(pose.y - circle.y, pose.x - circle.x)
    tangent = phase + _circle_direction(circle, pose) * pi / 2
    return (abs(hypot(pose.x - circle.x, pose.y - circle.y) - circle.radius) < 1e-5
            and abs(pose.z - circle.z) < 1e-5
            and abs((pose.yaw - tangent + pi) % tau - pi) < 1e-5)


def departure_phase_delay(request, route):
    """Time from the measured initial phase to the route's CCW departure point."""
    if request.initial_phase is None:
        return 0.0  # Legacy offline planner chooses its initial phase freely.
    p, c = route.paths[0].start, request.start
    angle = (atan2(p.y - c.y, p.x - c.x) - request.initial_phase) % tau
    if min(angle, tau - angle) < 1e-12:
        angle = 0.0
    return angle * c.radius / request.speed


@dataclass(frozen=True)
class ScheduledFlight:
    request: AgentRequest
    route: PlanningResult
    departure: float = 0.0

    def __post_init__(self):
        if not self.route.paths or not isfinite(self.departure) or self.departure < 0:
            raise ValueError("a scheduled flight needs a route and a finite nonnegative departure")
        if self.route.start_loiter != self.request.start or self.route.goal_loiter != self.request.goal:
            raise ValueError("route and request loiters differ")
        if not (_on_circle(self.request.start, self.route.paths[0].start)
                and _on_circle(self.request.goal, self.route.paths[-1].goal)):
            raise ValueError("route endpoints must be tangent states on their loiters")
        if _circle_direction(self.request.start, self.route.paths[0].start) != 1:
            raise ValueError("fleet start loiters use CCW departures")
        for a, b in zip(self.route.paths, self.route.paths[1:]):
            if (hypot(a.goal.x - b.start.x, a.goal.y - b.start.y, a.goal.z - b.start.z) > 1e-5
                    or abs((a.goal.yaw - b.start.yaw + pi) % tau - pi) > 1e-5):
                raise ValueError("route legs must join continuously")
        for p in self.route.paths:
            values = (p.start.x, p.start.y, p.start.z, p.start.yaw,
                      p.goal.x, p.goal.y, p.goal.z, p.goal.yaw,
                      p.radius, p.length, p.flight_path_angle)
            if (not all(isfinite(v) for v in values) or p.length < 0
                    or p.radius < self.request.radius - 1e-9
                    or abs(p.flight_path_angle) > self.request.max_angle + 1e-9
                    or len(p.segments) > 4
                    or any(s.kind not in ("L", "R", "S") or not isfinite(s.horizontal_length)
                           or s.horizontal_length < 0 for s in p.segments)):
                raise ValueError("route violates agent geometry limits")
        laps = (self.departure - self.phase_delay) / self.lap_period
        if laps < -1e-8 or abs(laps - round(laps)) > 1e-8:
            raise ValueError("departure must match the initial phase plus whole start-loiter laps")
        if self.departure + 1e-8 < self.request.earliest_departure:
            raise ValueError("departure is earlier than requested")

    @property
    def lap_period(self):
        return tau * self.request.start.radius / self.request.speed

    @property
    def waiting_laps(self):
        return round((self.departure - self.phase_delay) / self.lap_period)

    @property
    def phase_delay(self):
        return departure_phase_delay(self.request, self.route)

    @property
    def duration(self):
        return self.route.length / self.request.speed

    @property
    def arrival(self):
        return self.departure + self.duration

    def _loiter_state(self, circle, endpoint, elapsed):
        direction = _circle_direction(circle, endpoint)
        phase = atan2(endpoint.y - circle.y, endpoint.x - circle.x)
        phase += direction * self.request.speed * elapsed / circle.radius
        return Pose(circle.x + circle.radius * cos(phase),
                    circle.y + circle.radius * sin(phase), circle.z,
                    (phase + direction * pi / 2) % tau), 0.0

    def state_at(self, time):
        """Exact (Pose, flight-path angle) on a shared clock; valid after arrival too."""
        if not isfinite(time) or time < 0:
            raise ValueError("time must be finite and nonnegative")
        if time < self.departure:
            return self._loiter_state(self.request.start, self.route.paths[0].start,
                                      time - self.departure)
        if time >= self.arrival:
            return self._loiter_state(self.request.goal, self.route.paths[-1].goal,
                                      time - self.arrival)
        distance = (time - self.departure) * self.request.speed
        for path in self.route.paths:
            if distance <= path.length:
                return (path.pose_at(min(path.horizontal_length,
                                         distance * path.horizontal_length / path.length))
                        if path.length else path.start), path.flight_path_angle
            distance -= path.length
        return self.route.paths[-1].goal, 0.0  # Floating-point endpoint guard.

    def pose_at(self, time):
        return self.state_at(time)[0]


@dataclass(frozen=True)
class SeparationCheck:
    safe: bool
    time: float | None = None
    distance: float | None = None
    uncertain: bool = False  # Tight interval rejected conservatively, not a measured collision.


def _separation_python(a, b, separation, begin, end, step, tolerance):
    """Interval proof using |d(t)-d(s)| <= (Va+Vb)|t-s|, not frame sampling."""
    speed_bound = a.request.speed + b.request.speed
    threshold = separation + 1e-4  # Numerical margin, in metres.

    def distance(time):
        p, q = a.pose_at(time), b.pose_at(time)
        return hypot(p.x - q.x, p.y - q.y, p.z - q.z)

    def interval(left, right, dl, dr):
        middle = (left + right) / 2
        dm = distance(middle)
        closest, at = min((dl, left), (dm, middle), (dr, right))
        if closest <= threshold:
            return SeparationCheck(False, at, closest)
        # Every point is within dt/4 of one of these three samples.
        if closest - speed_bound * (right - left) / 4 > threshold:
            return SeparationCheck(True)
        if right - left <= tolerance or middle in (left, right):
            return SeparationCheck(False, at, closest, True)
        first = interval(left, middle, dl, dm)
        return first if not first.safe else interval(middle, right, dm, dr)

    left, dl = begin, distance(begin)
    if dl <= threshold:
        return SeparationCheck(False, left, dl)
    while left < end:
        right = min(end, left + step)
        if right <= left:
            return SeparationCheck(False, left, dl, True)
        dr = distance(right)
        result = interval(left, right, dl, dr)
        if not result.safe:
            return result
        left, dl = right, dr
    return SeparationCheck(True)


def check_separation(a, b, separation, end, *, begin=0.0, step=2.0, tolerance=0.01):
    """Continuously certify center-distance > separation over [begin, end].

    A tight interval that cannot be certified is rejected, even if its
    sampled distances happen to look safe. Results are conservative.
    Aircraft body sizes/tracking error must be included in `separation`.
    """
    if (not all(isfinite(v) for v in (separation, begin, end, step, tolerance))
            or separation <= 0 or begin < 0 or end < begin or not 0 < tolerance <= step):
        raise ValueError("invalid separation, time interval, step or tolerance")
    from .native import get_library, check_flight_pair
    lib = get_library()
    if lib is not None:
        return check_flight_pair(lib, a, b, separation, begin, end, step, tolerance)
    return _separation_python(a, b, separation, begin, end, step, tolerance)


def loiter_disk_distance(a, b):
    """Lower bound between all points of two level loiter disks."""
    horizontal = max(0.0, hypot(a.x - b.x, a.y - b.y) - a.radius - b.radius)
    return hypot(horizontal, a.z - b.z)


def sample_flight(flight, end, fps=10.0):
    """Exact seven-column poses on the fleet clock, with all route joins."""
    if not isfinite(end) or end <= 0 or not isfinite(fps) or fps <= 0:
        raise ValueError("sample end/fps must be positive and finite")
    count = max(1, ceil(end * fps))
    times = {end * i / count for i in range(count + 1)}
    elapsed = flight.departure
    times.add(elapsed)
    for path in flight.route.paths:
        for segment in path.segments:
            elapsed += segment.horizontal_length / cos(path.flight_path_angle) / flight.request.speed
            times.add(min(flight.arrival, elapsed))
    times.add(flight.arrival)
    rows = []
    for at in sorted(t for t in times if 0 <= t <= end):
        if rows and at - rows[-1][0] < 1e-10:
            continue
        pose, gamma = flight.state_at(at)
        rows.append([at, pose.x, pose.y, pose.z, 0.0, -gamma, pose.yaw])
    return rows


@dataclass(frozen=True)
class FleetPlan:
    flights: tuple[ScheduledFlight, ...]  # Request order is the priority order.
    separation: float
    horizon: float
    routing_ms: float
    scheduling_ms: float

    def __post_init__(self):
        if (not self.flights or len({f.request.name for f in self.flights}) != len(self.flights)
                or not all(isfinite(v) for v in
                           (self.separation, self.horizon, self.routing_ms, self.scheduling_ms))
                or self.separation <= 0 or self.horizon < self.makespan
                or min(self.routing_ms, self.scheduling_ms) < 0):
            raise ValueError("invalid fleet flights, horizon, separation or timings")

    @property
    def makespan(self):
        return max(f.arrival for f in self.flights)

    @property
    def planning_ms(self):
        return self.routing_ms + self.scheduling_ms

    def validate(self, world):
        """Independent final static + temporal checks; no agents disappear."""
        for f in self.flights:
            if not world.validate_loiter(f.request.start) or not world.validate_loiter(f.request.goal):
                return False
            if not all(world.path_is_free(p) for p in f.route.paths):
                return False
        for i, a in enumerate(self.flights):
            for b in self.flights[:i]:
                if (loiter_disk_distance(a.request.goal, b.request.goal) <= self.separation + 1e-4
                        or not check_separation(a, b, self.separation, self.horizon).safe):
                    return False
        return self.horizon >= self.makespan


def plan_fleet(world: FlightWorld, requests: Sequence[AgentRequest], *,
               separation=40.0, max_delay=120.0, iterations=600, seed=7) -> FleetPlan:
    """Retain full-budget routes, then try complete waiting laps.

    List order sets priority. All agents exist from t=0, including those
    waiting on their start circles. Arrival circles must be separated by
    whole disks: after the common validation horizon this certifies safe
    indefinite terminal loitering. Shared destinations/landing, spatial
    rerouting on conflict and priority backtracking are not implemented.
    NoScheduleError is not proof that a different joint plan is impossible.
    With initial_phase, first fly the partial circle to the chosen departure
    point, then add complete laps; legacy None leaves the initial phase free.
    """
    requests = tuple(requests)
    if (not requests or len({r.name for r in requests}) != len(requests)
            or not all(isfinite(v) for v in (separation, max_delay))
            or separation <= 0 or max_delay < 0
            or not isinstance(iterations, int) or iterations <= 0):
        raise ValueError("need unique agents, positive separation/iterations and nonnegative max_delay")
    for i, a in enumerate(requests):
        if a.earliest_departure > max_delay:
            raise ValueError("earliest departure exceeds max_delay")
        if not world.validate_loiter(a.start) or not world.validate_loiter(a.goal):
            raise ValueError(f"{a.name}: both loiter disks must be statically safe")
        for b in requests[:i]:
            if loiter_disk_distance(a.goal, b.goal) <= separation + 1e-4:
                raise ValueError(f"{a.name}/{b.name}: terminal loiter disks are not separated")
    began = perf_counter()
    routes = [plan_safe_rrt_star(world, r.start, r.goal, r.radius, r.max_angle,
                                iterations=iterations, seed=seed + i, start_direction="CCW")
              for i, r in enumerate(requests)]
    routing_ms = 1000 * (perf_counter() - began)
    began = perf_counter()
    horizon = max_delay + max(p.length / r.speed for r, p in zip(requests, routes))
    horizon += max(tau * r.goal.radius / r.speed for r in requests)
    chosen = []
    for r, route in zip(requests, routes):
        period = tau * r.start.radius / r.speed
        base = departure_phase_delay(r, route)
        first = max(0, ceil((r.earliest_departure - base) / period))
        last = floor((max_delay - base) / period)
        for laps in range(first, last + 1):
            candidate = ScheduledFlight(r, route, base + laps * period)
            if all(check_separation(candidate, old, separation, horizon).safe for old in chosen):
                chosen.append(candidate)
                break
        else:
            raise NoScheduleError(f"{r.name}: no safe departure in {max_delay:g} s for these routes/priority; "
                                  "increase max_delay, change priority/seed or separate the loiters")
    result = FleetPlan(tuple(chosen), separation, horizon, routing_ms, 1000 * (perf_counter() - began))
    if not result.validate(world):
        raise RuntimeError("final static/continuous fleet validation failed")
    return result
