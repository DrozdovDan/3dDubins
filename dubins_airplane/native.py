"""ctypes adapter: steering, collision checks and tree search stay in C++.

DUBINS_BACKEND=auto (default), cpp, or python selects the implementation.
Input/output conversion, seeded sampling and a first-solution callback stay
in Python; there are no per-edge Python callbacks in the search loop.
"""

from __future__ import annotations

import ctypes as ct
from functools import lru_cache
import os
from pathlib import Path
from random import Random
import sys
from time import perf_counter


class _Pose(ct.Structure):
    _fields_ = [(key, ct.c_double) for key in ("x", "y", "z", "yaw")]


class _Segment(ct.Structure):
    _fields_ = [("length", ct.c_double), ("kind", ct.c_int)]


class _Path(ct.Structure):
    _fields_ = [("start", _Pose), ("goal", _Pose)] + [
        (key, ct.c_double) for key in ("radius", "minimum_radius", "max_angle", "gamma",
                                       "extension", "horizontal", "length")
    ] + [("segments", _Segment * 4)] + [
        (key, ct.c_int) for key in ("count", "turns", "altitude_case", "base_word")
    ] + [("word", ct.c_char * 5)]


class _Box(ct.Structure):
    _fields_ = [(key, ct.c_double) for key in ("x0", "y0", "z0", "x1", "y1", "z1")]


class _World(ct.Structure):
    _fields_ = [(key, ct.c_double) for key in ("x0", "y0", "resolution", "margin")]
    _fields_ += [(key, ct.c_int) for key in ("rows", "columns", "boxes_count")]
    _fields_ += [("lower", ct.POINTER(ct.c_double)), ("upper", ct.POINTER(ct.c_double)),
                ("boxes", ct.POINTER(_Box))]


class _Options(ct.Structure):
    _fields_ = [(key, ct.c_double) for key in ("radius", "max_angle", "neighbour_radius",
                                             "collision_step", "time_limit")]


class _History(ct.Structure):
    _fields_ = [("iteration", ct.c_int), ("cost", ct.c_double)]


_Callback = ct.CFUNCTYPE(ct.c_int, ct.c_int)
_library = None


@lru_cache(maxsize=1)
def library_path():
    suffix = ".dll" if sys.platform == "win32" else ".dylib" if sys.platform == "darwin" else ".so"
    return Path(__file__).with_name("_native" + suffix)


@lru_cache(maxsize=1)
def _load(filename):
    lib = ct.CDLL(filename)
    lib.dubins_abi_version.restype = ct.c_int
    if lib.dubins_abi_version() != 1:
        raise RuntimeError("native ABI mismatch; rebuild with python -m dubins_airplane.build_native")
    lib.dubins_error.restype = ct.c_char_p
    lib.dubins_plan.argtypes = [ct.POINTER(_Pose), ct.POINTER(_Pose), ct.c_double, ct.c_double,
                               ct.POINTER(_Path)]
    lib.dubins_check.argtypes = [ct.POINTER(_World), ct.POINTER(_Path), ct.c_double, ct.c_double]
    lib.dubins_offset.argtypes = [ct.POINTER(ct.c_double), ct.c_int, ct.c_int, ct.c_double,
                                 ct.c_double, ct.c_int, ct.POINTER(ct.c_double)]
    lib.dubins_rrt.argtypes = [ct.POINTER(_World), ct.POINTER(_Pose), ct.c_int,
                              ct.POINTER(_Pose), ct.c_int, ct.POINTER(_Pose), ct.c_int,
                              ct.POINTER(_Options), ct.POINTER(_Path), ct.POINTER(ct.c_int),
                              ct.POINTER(_History), ct.POINTER(ct.c_int), ct.POINTER(ct.c_int),
                              ct.POINTER(ct.c_int), ct.POINTER(ct.c_int), _Callback]
    return lib


def get_library():
    global _library
    backend = os.environ.get("DUBINS_BACKEND", "auto").lower()
    if backend not in ("auto", "cpp", "python"):
        raise ValueError("DUBINS_BACKEND must be auto, cpp or python")
    if backend == "python":
        return None
    # Do not stat the filesystem on every individual steering call. An
    # already-loaded DLL cannot be hot-swapped safely anyway. Missing builds
    # are not cached, so a later build in the same process is still detected.
    if _library is not None:
        return _library
    filename = library_path()
    if filename.is_file():
        _library = _load(str(filename))
        return _library
    if backend == "cpp":
        raise RuntimeError("C++ backend is not built; run python -m dubins_airplane.build_native")
    return None


def backend_name():
    return "cpp" if get_library() is not None else "python"


def _check(lib, status):
    if status:
        message = lib.dubins_error().decode("utf-8", errors="replace")
        raise (ValueError if status == 1 else RuntimeError)(message)


def _pose(p):
    return _Pose(p.x, p.y, p.z, p.yaw)


def _python_path(p):
    from .core import DubinsAirplanePath, Pose, Segment, CSC_WORDS
    return DubinsAirplanePath(
        Pose(p.start.x, p.start.y, p.start.z, p.start.yaw),
        Pose(p.goal.x, p.goal.y, p.goal.z, p.goal.yaw), p.radius, p.max_angle, p.gamma,
        p.word.decode("ascii"), p.turns,
        tuple(Segment("LRS"[s.kind], s.length) for s in p.segments[:p.count]),
        p.minimum_radius, ("low", "medium", "high")[p.altitude_case],
        CSC_WORDS[p.base_word], p.extension,
    )


def _native_path(p):
    out = _Path()
    out.start, out.goal = _pose(p.start), _pose(p.goal)
    out.radius, out.minimum_radius, out.max_angle = p.radius, p.minimum_radius, p.max_flight_path_angle
    out.gamma, out.extension = p.flight_path_angle, p.extension_angle
    out.horizontal, out.length = p.horizontal_length, p.length
    out.count, out.turns = len(p.segments), p.extra_turns
    if out.count > 4:
        raise ValueError("native checker supports Owen paths with at most four segments")
    for i, segment in enumerate(p.segments):
        out.segments[i] = _Segment(segment.horizontal_length, "LRS".index(segment.kind))
    return out


def _world(w):
    grid = w.terrain
    array = ct.c_double * (grid.rows * grid.columns)
    lower = array(*(value for row in w.lower for value in row))
    upper = array(*(value for row in w.upper for value in row))
    boxes = (_Box * len(w.obstacles))(*(_Box(b.x_min, b.y_min, b.z_min, b.x_max, b.y_max, b.z_max)
                                      for b in w.obstacles))
    out = _World(grid.x_min, grid.y_min, grid.resolution, w.obstacle_margin,
                 grid.rows, grid.columns, len(boxes), lower, upper, boxes)
    return out, (lower, upper, boxes)  # keep borrowed buffers alive


def plan(lib, start, goal, radius, max_angle):
    a, b, out = _pose(start), _pose(goal), _Path()
    _check(lib, lib.dubins_plan(ct.byref(a), ct.byref(b), radius, max_angle, ct.byref(out)))
    return _python_path(out)


def offset(lib, grid, distance, upper_bound):
    array = ct.c_double * (grid.rows * grid.columns)
    heights = array(*(value for row in grid.heights for value in row))
    out = array()
    _check(lib, lib.dubins_offset(heights, grid.rows, grid.columns, grid.resolution,
                                 distance, upper_bound, out))
    return tuple(tuple(out[r * grid.columns + c] for c in range(grid.columns))
                 for r in range(grid.rows))


def path_is_free(lib, world, path, max_step, min_step):
    data, keepalive = _world(world)
    p = _native_path(path)
    status = lib.dubins_check(ct.byref(data), ct.byref(p), max_step, min_step)
    if status < 0:
        _check(lib, -status)
    return bool(status)


def plan_rrt(lib, world, start, goal, radius, max_angle, *, iterations, seed,
             circle_samples, start_direction, goal_bias, neighbor_radius,
             collision_step, time_limit, on_first_solution):
    from math import nan, pi
    from .core import Pose
    from .rrt_star import NoPathError, PlanningResult, _validate_inputs
    _validate_inputs(world, start, goal, radius, max_angle, iterations, circle_samples,
                     goal_bias, neighbor_radius, collision_step, time_limit)
    began = perf_counter()
    roots = start.states(circle_samples, start_direction)
    goals = goal.states(circle_samples, "CW") + goal.states(circle_samples, "CCW")
    rng = Random(seed)
    grid = world.terrain
    samples = []
    for _ in range(iterations):
        if rng.random() < goal_bias:
            sampled = rng.choice(goals)
        else:
            x, y = rng.uniform(grid.x_min, grid.x_max), rng.uniform(grid.y_min, grid.y_max)
            band = world.altitude_band(x, y)
            if band is None:
                samples.append(_Pose(nan, nan, nan, nan))
                continue
            sampled = Pose(x, y, rng.uniform(*band), rng.uniform(-pi, pi))
        samples.append(_pose(sampled))
    data, keepalive = _world(world)
    root_data = (_Pose * len(roots))(*map(_pose, roots))
    goal_data = (_Pose * len(goals))(*map(_pose, goals))
    sample_data = (_Pose * iterations)(*samples)
    remaining = max(0.0, time_limit - (perf_counter() - began)) if time_limit is not None else -1
    options = _Options(radius, max_angle, neighbor_radius, collision_step, remaining)
    paths, history = (_Path * (iterations + 1))(), (_History * (iterations + 1))()
    count, history_count, completed, nodes, rewires = (ct.c_int() for _ in range(5))
    callback_errors = []

    def notify(iteration):
        try:
            on_first_solution(iteration)
            return 0
        except BaseException as exc:
            callback_errors.append(exc)
            return 1

    callback = _Callback(notify) if on_first_solution is not None else _Callback()
    status = lib.dubins_rrt(ct.byref(data), root_data, len(roots), goal_data, len(goals),
                             sample_data, iterations, ct.byref(options), paths, ct.byref(count),
                             history, ct.byref(history_count), ct.byref(completed), ct.byref(nodes),
                             ct.byref(rewires), callback)
    if callback_errors:
        raise callback_errors[0]
    _check(lib, status)
    if not count.value:
        raise NoPathError(f"no route in {completed.value} iterations ({nodes.value} nodes); "
                          "increase the budget, change seed/loiters, or inspect the map")
    return PlanningResult(tuple(_python_path(p) for p in paths[:count.value]), start, goal,
                          completed.value, nodes.value, rewires.value,
                          tuple((h.iteration, h.cost) for h in history[:history_count.value]))
