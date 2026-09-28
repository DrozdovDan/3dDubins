"""Owen--Beard--McLain Dubins-airplane construction (2014), Section 4.

Low altitude: a CSC path at R_min with a fitted flight-path angle.
Medium altitude: an additional arc at R_min, fitted by bisection.
High altitude: complete turns and a fitted radius, Equation (20).

This is an independent implementation of the paper's geometric planner,
not its autopilot or vector-field guidance controller. Coordinates remain
east/north/up, with counterclockwise yaw from east (the paper uses NED).
Only the four CSC families used in the chapter are considered, not CCC.

Paper: https://scholarsarchive.byu.edu/facpub/1900/
Reference implementation: https://github.com/ntnu-arl/DubinsAirplane
The standard planar CSC formulas were checked against PythonRobotics.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, ceil, cos, floor, hypot, isfinite, pi, sin, sqrt, tan
from typing import Callable

TAU = 2.0 * pi
CSC_WORDS = ("RSR", "RSL", "LSR", "LSL")  # reference tie-breaking order


def minimum_turn_radius(airspeed: float, max_bank_angle: float,
                        gravity: float = 9.81) -> float:
    """Equation (19): R_min = V**2 / (g * tan(bank_max)), in metres.

    Airspeed is metres/second, bank angle radians, gravity metres/second**2.
    This is the radius convention used by the chapter's planner.
    """
    if (not all(isfinite(v) for v in (airspeed, max_bank_angle, gravity))
            or airspeed <= 0 or gravity <= 0
            or not 0 < max_bank_angle < pi / 2):
        raise ValueError("speed/gravity must be positive and bank angle in (0, pi/2)")
    return airspeed**2 / (gravity * tan(max_bank_angle))


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
    radius: float  # actual radius R*, which can exceed the input minimum
    max_flight_path_angle: float
    flight_path_angle: float
    word: str  # CSC for low/high; CCSC or CSCC for medium
    extra_turns: int
    segments: tuple[Segment, ...]
    minimum_radius: float
    altitude_case: str  # "low", "medium", or "high"
    base_word: str  # CSC family selected at R_min
    extension_angle: float = 0.0  # medium-case search parameter phi, radians

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
        for segment_index, segment in enumerate(self.segments):
            count = max(1, ceil(segment.horizontal_length / max_horizontal_step))
            for i in range(1, count + 1):
                distance = min(self.horizontal_length,
                               distance_done + segment.horizontal_length * i / count)
                if segment_index == len(self.segments) - 1 and i == count:
                    distance = self.horizontal_length
                points.append(self.pose_at(distance))
            distance_done += segment.horizontal_length
        return points


def _planar_words(start: Pose, goal: Pose, radius: float):
    """Yield the feasible CSC words from Section 4.1, with lengths in metres."""
    dx, dy = goal.x - start.x, goal.y - start.y
    if hypot(dx, dy) < 1e-12 * radius and abs(_angle_error(start.yaw, goal.yaw)) < 1e-12:
        yield "RSR", (0.0, 0.0, 0.0)
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


def _shortest_csc(start: Pose, goal: Pose, radius: float):
    """Re-evaluate L_car(R), with the reference's tie-breaking order."""
    candidates = dict(_planar_words(start, goal, radius))
    word = min((word for word in CSC_WORDS if word in candidates),
               key=lambda word: sum(candidates[word]))
    return word, candidates[word]


def _turn(pose: Pose, kind: str, radius: float, angle: float) -> Pose:
    """Advance on a horizontal circle; negative angle traces it backwards."""
    direction = 1 if kind == "L" else -1
    yaw = pose.yaw + direction * angle
    return Pose(pose.x + direction * radius * (sin(yaw) - sin(pose.yaw)),
                pose.y + direction * radius * (cos(pose.yaw) - cos(yaw)),
                pose.z, _wrap(yaw))


def _bisect(function: Callable[[float], float | None], low: float, high: float,
            tolerance: float) -> float | None:
    """Solve a bracketed length equation, rejecting infeasibility and wrap jumps."""
    f_low, f_high = function(low), function(high)
    if f_low is None or f_high is None:
        return None
    if abs(f_low) <= tolerance:
        return low
    if abs(f_high) <= tolerance:
        return high
    if f_low * f_high > 0:
        return None
    for _ in range(100):
        middle = (low + high) / 2
        value = function(middle)
        if value is None:
            return None
        if abs(value) <= tolerance:
            return middle
        if f_low * value <= 0:
            high = middle
        else:
            low, f_low = middle, value
    # A discontinuity in a wrapped arc is not a solution of the length equation.
    return None


def _find_root(function: Callable[[float], float | None], low: float, high: float,
               tolerance: float) -> float:
    """Bracket a continuous branch before bisection (including tangent boundaries)."""
    root = _bisect(function, low, high, tolerance)
    if root is not None:
        return root
    # Wrapped angles and disappearing internal tangents can split the interval.
    # Do not silently substitute full loops for Owen's intermediate arc.
    previous = low
    for i in range(1, 257):
        current = low + (high - low) * i / 256
        root = _bisect(function, previous, current, tolerance)
        if root is not None:
            return root
        previous = current
    raise ValueError(
        "Owen's selected CSC branch has no continuous length-equation solution. "
        "Nearby poses can require CCC paths, which this chapter does not cover; "
        "the reference examples use horizontal separation >= 6 * minimum radius."
    )


def _medium_path(start: Pose, goal: Pose, radius: float, base_word: str,
                 required: float, climb: bool):
    """Section 4.2.3: CCSC for climb, CSCC for descent at gamma_max.

    Advance along the original start circle (or backwards along the end
    circle), then re-plan L_car from that intermediate configuration. Its
    new first/last arc is the intermediate arc. Phi is the displacement
    angle, not necessarily the sweep of that new intermediate arc.
    """
    kind = base_word[0] if climb else base_word[-1]

    def candidate(phi):
        intermediate = _turn(start if climb else goal, kind, radius,
                             phi if climb else -phi)
        inner_word, lengths = _shortest_csc(intermediate if climb else start,
                                            goal if climb else intermediate, radius)
        return ((kind + inner_word, (radius * phi, *lengths)) if climb
                else (inner_word + kind, (*lengths, radius * phi)))

    def residual(phi):
        _, lengths = candidate(phi)
        return sum(lengths) - required

    tolerance = 1e-10 * max(1.0, required)
    phi = _find_root(residual, 0.0, TAU, tolerance)
    word, lengths = candidate(phi)
    return word, lengths, phi


def plan_dubins_airplane(
    start: Pose,
    goal: Pose,
    radius: float,
    max_flight_path_angle: float,
) -> DubinsAirplanePath:
    """Plan the three altitude cases in Owen--Beard--McLain, Section 4.2.

    Input radius is R_min in metres. Result.radius is the fitted R* in the
    high case. Angles are radians; z is upward. Low uses a fitted gamma;
    medium/high use +/-gamma_max to numerical length-solver tolerance.
    Extension is placed at the start for climb and at the end for descent.

    The chapter considers only CSC base paths. It is not a general six-word
    planar Dubins solver or a global optimizer over all 3D curves. A failed
    length equation (particularly for nearby poses) raises ValueError.
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
    required = abs(dz) / tan(max_flight_path_angle)
    base_word, lengths = _shortest_csc(start, goal, radius)
    base_length = sum(lengths)
    word = base_word
    actual_radius = radius
    turns = 0
    phi = 0.0
    circumference = TAU * radius
    if required <= base_length:
        altitude_case = "low"
    elif required < base_length + circumference:
        altitude_case = "medium"
        word, lengths, phi = _medium_path(start, goal, radius, base_word,
                                          required, dz > 0)
    else:
        altitude_case = "high"
        # Equation (20): use floor, not ceil; make up the remainder by R*.
        turns = max(1, floor((required - base_length) / circumference))

        def residual(trial_radius):
            # L_car(R) in Equation (20) is the shortest CSC at THAT radius.
            # Re-select the family: freezing it can create a 2*pi arc jump
            # and leave the length equation with no solution.
            _, trial_lengths = _shortest_csc(start, goal, trial_radius)
            return sum(trial_lengths) + TAU * turns * trial_radius - required

        actual_radius = _find_root(residual, radius, 2 * radius,
                                   1e-10 * max(1.0, required))
        word, lengths = _shortest_csc(start, goal, actual_radius)
        lengths = list(lengths)
        # Fold complete turns into the start/end helix rather than adding a
        # separate artificial segment. On descent use the END turn direction.
        lengths[0 if dz > 0 else -1] += TAU * turns * actual_radius

    segments = tuple(Segment(kind, length) for kind, length in zip(word, lengths)
                     if length > 1e-12)
    total = sum(segment.horizontal_length for segment in segments)

    gamma = atan2(dz, total) if total else 0.0
    path = DubinsAirplanePath(
        start=start, goal=goal, radius=actual_radius,
        max_flight_path_angle=max_flight_path_angle,
        flight_path_angle=gamma, word=word, extra_turns=turns,
        segments=segments, minimum_radius=radius,
        altitude_case=altitude_case, base_word=base_word, extension_angle=phi,
    )
    end = path.pose_at(path.horizontal_length)
    if (hypot(end.x - goal.x, end.y - goal.y) > 1e-7 * max(1.0, actual_radius)
            or abs(_angle_error(end.yaw, goal.yaw)) > 1e-8):
        raise RuntimeError("internal Dubins calculation did not reach the goal")
    return path
