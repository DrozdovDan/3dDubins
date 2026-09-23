"""Small, dependency-free 3D Dubins-airplane route-leg planner.

The state is (x, y, z, yaw), with yaw in radians. Turns have a minimum
radius in the horizontal plane, and the constant flight-path angle is
bounded. A whole circular helix is added when a planar path is too short
for the requested altitude change. See docs/KNOWLEDGE_GUIDE.md for the model,
sources, and limitations.

The six planar Dubins word formulas follow the standard construction; the
implementation was checked against the formulas in PythonRobotics:
https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/DubinsPath/dubins_path_planner.py
"""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, ceil, cos, hypot, isfinite, pi, sin, sqrt, tan

TAU = 2.0 * pi


def _wrap(angle: float) -> float:
    value = angle % TAU
    return 0.0 if TAU - value < 1e-12 else value


def _angle_error(a: float, b: float) -> float:
    return (a - b + pi) % TAU - pi


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    yaw: float  # radians, measured counterclockwise from +x


@dataclass(frozen=True)
class Segment:
    kind: str  # L: left, R: right, S: straight
    horizontal_length: float  # metres along the horizontal projection


@dataclass(frozen=True)
class DubinsAirplanePath:
    start: Pose
    goal: Pose
    radius: float
    max_flight_path_angle: float
    flight_path_angle: float
    word: str  # planar Dubins word before any complete helical turns
    extra_turns: int
    segments: tuple[Segment, ...]

    @property
    def horizontal_length(self) -> float:
        return sum(segment.horizontal_length for segment in self.segments)

    @property
    def length(self) -> float:
        """Length along the 3D path in metres."""
        return hypot(self.horizontal_length, self.goal.z - self.start.z)

    def pose_at(self, horizontal_distance: float) -> Pose:
        """Exact pose at a distance along the horizontal projection."""
        total = self.horizontal_length
        if not isfinite(horizontal_distance) or not 0.0 <= horizontal_distance <= total:
            raise ValueError("horizontal_distance must lie on the path")
        if total == 0.0:
            return self.start

        x, y, yaw = self.start.x, self.start.y, self.start.yaw
        remaining = horizontal_distance
        for segment in self.segments:
            distance = min(remaining, segment.horizontal_length)
            if segment.kind == "S":
                x += distance * cos(yaw)
                y += distance * sin(yaw)
            elif segment.kind == "L":
                next_yaw = yaw + distance / self.radius
                x += self.radius * (sin(next_yaw) - sin(yaw))
                y += self.radius * (cos(yaw) - cos(next_yaw))
                yaw = next_yaw
            else:  # R
                next_yaw = yaw - distance / self.radius
                x += self.radius * (sin(yaw) - sin(next_yaw))
                y += self.radius * (cos(next_yaw) - cos(yaw))
                yaw = next_yaw
            remaining -= distance
            if remaining <= 0.0:
                break

        z = self.start.z + (self.goal.z - self.start.z) * horizontal_distance / total
        return Pose(x, y, z, _wrap(yaw))

    def sample(self, max_horizontal_step: float = 5.0) -> list[Pose]:
        """Sample the path, including the start, goal, and every segment end."""
        if not isfinite(max_horizontal_step) or max_horizontal_step <= 0.0:
            raise ValueError("max_horizontal_step must be finite and positive")
        points = [self.pose_at(0.0)]
        distance_done = 0.0
        for segment in self.segments:
            count = max(1, ceil(segment.horizontal_length / max_horizontal_step))
            for i in range(1, count + 1):
                distance = min(self.horizontal_length,
                               distance_done + segment.horizontal_length * i / count)
                points.append(self.pose_at(distance))
            distance_done += segment.horizontal_length
        if self.horizontal_length == 0.0:
            return points
        return points


def _planar_words(start: Pose, goal: Pose, radius: float):
    """Yield feasible (word, three horizontal segment lengths)."""
    dx, dy = goal.x - start.x, goal.y - start.y
    if hypot(dx, dy) < 1e-12 * radius and abs(_angle_error(start.yaw, goal.yaw)) < 1e-12:
        yield "LSL", (0.0, 0.0, 0.0)
        return
    theta = atan2(dy, dx)
    d = hypot(dx, dy) / radius
    alpha = _wrap(start.yaw - theta)
    beta = _wrap(goal.yaw - theta)
    sa, sb = sin(alpha), sin(beta)
    ca, cb = cos(alpha), cos(beta)
    cab = cos(alpha - beta)

    def straight(square: float) -> float | None:
        return sqrt(max(0.0, square)) if square >= -1e-12 else None

    p = straight(2 + d * d - 2 * cab + 2 * d * (sa - sb))
    if p is not None:
        t = atan2(cb - ca, d + sa - sb)
        yield "LSL", (_wrap(-alpha + t) * radius, p * radius, _wrap(beta - t) * radius)

    p = straight(2 + d * d - 2 * cab + 2 * d * (sb - sa))
    if p is not None:
        t = atan2(ca - cb, d - sa + sb)
        yield "RSR", (_wrap(alpha - t) * radius, p * radius, _wrap(-beta + t) * radius)

    p = straight(-2 + d * d + 2 * cab + 2 * d * (sa + sb))
    if p is not None:
        t = atan2(-ca - cb, d + sa + sb) - atan2(-2.0, p)
        yield "LSR", (_wrap(-alpha + t) * radius, p * radius, _wrap(-beta + t) * radius)

    p = straight(-2 + d * d + 2 * cab - 2 * d * (sa + sb))
    if p is not None:
        t = atan2(ca + cb, d - sa - sb) - atan2(2.0, p)
        yield "RSL", (_wrap(alpha - t) * radius, p * radius, _wrap(beta - t) * radius)

    value = (6 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8
    if -1 - 1e-12 <= value <= 1 + 1e-12:
        middle = _wrap(TAU - acos(max(-1.0, min(1.0, value))))
        first = _wrap(alpha - atan2(ca - cb, d - sa + sb) + middle / 2)
        last = _wrap(alpha - beta - first + middle)
        yield "RLR", (first * radius, middle * radius, last * radius)

    value = (6 - d * d + 2 * cab + 2 * d * (-sa + sb)) / 8
    if -1 - 1e-12 <= value <= 1 + 1e-12:
        middle = _wrap(TAU - acos(max(-1.0, min(1.0, value))))
        first = _wrap(-alpha - atan2(ca - cb, d + sa - sb) + middle / 2)
        last = _wrap(beta - alpha - first + middle)
        yield "LRL", (first * radius, middle * radius, last * radius)


def plan_dubins_airplane(
    start: Pose,
    goal: Pose,
    radius: float,
    max_flight_path_angle: float,
) -> DubinsAirplanePath:
    """Connect two (x, y, z, yaw) poses with a feasible 3D aircraft path.

    x/y/z/radius are metres; angles are radians. The path has constant
    flight-path angle. It chooses the shortest among six planar Dubins
    words, each with as many complete helical turns as needed to respect
    the climb/descent angle limit. It is not a globally shortest 3D path.
    """
    numbers = (start.x, start.y, start.z, start.yaw, goal.x, goal.y,
               goal.z, goal.yaw, radius, max_flight_path_angle)
    if not all(isfinite(value) for value in numbers):
        raise ValueError("all inputs must be finite")
    if radius <= 0.0:
        raise ValueError("radius must be positive")
    if not 0.0 < max_flight_path_angle < pi / 2:
        raise ValueError("max_flight_path_angle must lie between 0 and pi/2")

    dz = goal.z - start.z
    min_horizontal = abs(dz) / tan(max_flight_path_angle)
    best = None
    circumference = TAU * radius
    for word, lengths in _planar_words(start, goal, radius):
        base_length = sum(lengths)
        shortage = max(0.0, min_horizontal - base_length)
        turns = max(0, ceil((shortage - 1e-10 * max(1.0, min_horizontal)) / circumference))
        total = base_length + turns * circumference
        if total + 1e-9 < min_horizontal:
            turns += 1
            total += circumference
        candidate = (total, turns, word, lengths)
        if best is None or candidate[:2] < best[:2]:
            best = candidate

    if best is None:
        raise RuntimeError("no planar Dubins path found")
    total, turns, word, lengths = best
    base_segments = [Segment(kind, length) for kind, length in zip(word, lengths) if length > 1e-12]
    if turns:
        turn_kind = word[0]  # a full circle returns to the start pose
        loop = Segment(turn_kind, turns * circumference)
        segments = [loop, *base_segments] if dz >= 0 else [*base_segments, loop]
    else:
        segments = base_segments

    gamma = atan2(dz, total) if total else 0.0
    path = DubinsAirplanePath(start, goal, radius, max_flight_path_angle,
                              gamma, word, turns, tuple(segments))
    end = path.pose_at(path.horizontal_length)
    if (hypot(end.x - goal.x, end.y - goal.y) > 1e-7 * max(1.0, radius)
            or abs(_angle_error(end.yaw, goal.yaw)) > 1e-8):
        raise RuntimeError("internal Dubins calculation did not reach the goal")
    return path
