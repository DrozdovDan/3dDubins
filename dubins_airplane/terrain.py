"""Raster terrain, collision bands and safe loiters from Lim et al. (2024).

Independent, standard-library implementation of Sections III-B and IV-C.
Cells are horizontal, constant-height terrain patches, not point samples.
Offset bands conservatively bound each WHOLE cell. This matters near cliffs.
Coordinates are local east/north/up metres, as in core.py.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, cos, floor, hypot, isfinite, pi, sin, sqrt, tau
from typing import Callable

from .core import DubinsAirplanePath, Pose


@dataclass(frozen=True)
class ElevationMap:
    """heights[row][column], with rows increasing north and columns east."""

    x_min: float
    y_min: float
    resolution: float
    heights: tuple[tuple[float, ...], ...]

    def __post_init__(self):
        if (not all(isfinite(v) for v in (self.x_min, self.y_min, self.resolution))
                or self.resolution <= 0 or not self.heights or not self.heights[0]):
            raise ValueError("map needs a finite origin, positive resolution and nonempty rows")
        width = len(self.heights[0])
        if any(len(row) != width or not all(isfinite(v) for v in row) for row in self.heights):
            raise ValueError("terrain heights must form a rectangular finite array")

    @classmethod
    def from_function(cls, x_min: float, y_min: float, resolution: float,
                      columns: int, rows: int, height: Callable[[float, float], float]):
        """Sample a synthetic height field at cell centres; not a GIS importer."""
        if not isinstance(columns, int) or not isinstance(rows, int) or min(columns, rows) <= 0:
            raise ValueError("rows and columns must be positive integers")
        return cls(x_min, y_min, resolution,
                   tuple(tuple(height(x_min + (col + 0.5) * resolution,
                                      y_min + (row + 0.5) * resolution)
                               for col in range(columns)) for row in range(rows)))

    @property
    def columns(self):
        return len(self.heights[0])

    @property
    def rows(self):
        return len(self.heights)

    @property
    def x_max(self):
        return self.x_min + self.columns * self.resolution

    @property
    def y_max(self):
        return self.y_min + self.rows * self.resolution

    def cell(self, x: float, y: float) -> tuple[int, int] | None:
        if not (isfinite(x) and isfinite(y) and self.x_min <= x < self.x_max
                and self.y_min <= y < self.y_max):
            return None
        return (floor((y - self.y_min) / self.resolution),
                floor((x - self.x_min) / self.resolution))

    def height_at(self, x: float, y: float) -> float:
        cell = self.cell(x, y)
        if cell is None:
            raise ValueError("position is outside the known terrain map")
        row, col = cell
        return self.heights[row][col]


@dataclass(frozen=True)
class BoxObstacle:
    """Closed axis-aligned forbidden volume; includes its boundary."""

    x_min: float
    y_min: float
    z_min: float
    x_max: float
    y_max: float
    z_max: float

    def __post_init__(self):
        if (not all(isfinite(v) for v in (self.x_min, self.y_min, self.z_min,
                                         self.x_max, self.y_max, self.z_max))
                or self.x_min >= self.x_max or self.y_min >= self.y_max
                or self.z_min >= self.z_max):
            raise ValueError("obstacle needs finite, strictly increasing bounds")

    def intersects(self, x0, y0, z0, x1, y1, z1, margin=0.0):
        return (x1 >= self.x_min - margin and x0 <= self.x_max + margin
                and y1 >= self.y_min - margin and y0 <= self.y_max + margin
                and z1 >= self.z_min - margin and z0 <= self.z_max + margin)


@dataclass(frozen=True)
class LoiterCircle:
    """A level circle. ENU yaw is tangent, not the centre's polar angle."""

    x: float
    y: float
    z: float
    radius: float

    def __post_init__(self):
        if not all(isfinite(v) for v in (self.x, self.y, self.z, self.radius)) or self.radius <= 0:
            raise ValueError("loiter must have finite coordinates and a positive radius")

    def states(self, count=16, direction="CCW") -> tuple[Pose, ...]:
        if not isinstance(count, int) or count < 4 or direction not in ("CW", "CCW"):
            raise ValueError("use at least four circle states and direction CW or CCW")
        sign = 1 if direction == "CCW" else -1
        return tuple(Pose(self.x + self.radius * cos(tau * i / count),
                          self.y + self.radius * sin(tau * i / count), self.z,
                          (tau * i / count + sign * pi / 2) % tau) for i in range(count))


class FlightWorld:
    """Lim's flight band plus optional box obstacles (an additional feature).

    min_clearance/max_clearance are EUCLIDEAN offsets, not merely H(x,y)+d.
    Known cells bound a piecewise-constant terrain solid. The floor uses an
    upper bound on D_min; the ceiling a lower bound on D_max, throughout
    each cell. Resolution can therefore exclude narrow but truly free gaps.
    obstacle_margin inflates boxes in all three axes, conservatively.
    """

    def __init__(self, terrain: ElevationMap, min_clearance: float,
                 max_clearance: float, obstacles=(), obstacle_margin=0.0):
        if (not all(isfinite(v) for v in (min_clearance, max_clearance, obstacle_margin))
                or not 0 <= min_clearance < max_clearance or obstacle_margin < 0):
            raise ValueError("require 0 <= min clearance < max clearance and nonnegative margin")
        self.terrain = terrain
        self.min_clearance, self.max_clearance = min_clearance, max_clearance
        self.obstacles = tuple(obstacles)
        self.obstacle_margin = obstacle_margin
        self.lower = self._offset(min_clearance, upper_bound=True)
        self.upper = self._offset(max_clearance, upper_bound=False)

    def _offset(self, distance, upper_bound):
        # Equation (4) is a spherical dilation. For cells, use the nearest
        # rectangle distance for a conservative floor and the farthest
        # target-to-source distance for a conservative ceiling.
        grid = self.terrain
        reach = ceil(distance / grid.resolution) + 1
        kernel = []
        for dr in range(-reach, reach + 1):
            for dc in range(-reach, reach + 1):
                if upper_bound:
                    dx, dy = max(0, abs(dc) - 1), max(0, abs(dr) - 1)
                else:
                    dx, dy = abs(dc), abs(dr)
                lateral = hypot(dx, dy) * grid.resolution
                if lateral <= distance:
                    kernel.append((dr, dc, sqrt(max(0, distance**2 - lateral**2))))
        return tuple(tuple(max(grid.heights[row + dr][col + dc] + dz
                               for dr, dc, dz in kernel
                               if 0 <= row + dr < grid.rows and 0 <= col + dc < grid.columns)
                           for col in range(grid.columns)) for row in range(grid.rows))

    def altitude_band(self, x, y) -> tuple[float, float] | None:
        cell = self.terrain.cell(x, y)
        if cell is None:
            return None
        row, col = cell
        low, high = self.lower[row][col], self.upper[row][col]
        return (low, high) if low < high else None

    def is_free(self, pose: Pose) -> bool:
        if not all(isfinite(v) for v in (pose.x, pose.y, pose.z, pose.yaw)):
            return False
        band = self.altitude_band(pose.x, pose.y)
        return (band is not None and band[0] < pose.z < band[1]
                and not any(box.intersects(pose.x, pose.y, pose.z, pose.x, pose.y, pose.z,
                                           self.obstacle_margin) for box in self.obstacles))

    def _cells_in_rectangle(self, x0, y0, x1, y1):
        first, last = self.terrain.cell(x0, y0), self.terrain.cell(x1, y1)
        if first is None or last is None:
            return None
        return [(row, col) for row in range(first[0], last[0] + 1)
                for col in range(first[1], last[1] + 1)]

    def loiter_band(self, x, y, radius) -> tuple[float, float] | None:
        """Disk max/min filters: Equation (7). Unknown map space is forbidden."""
        if not all(isfinite(v) for v in (x, y, radius)) or radius <= 0:
            raise ValueError("loiter position/radius must be finite, radius positive")
        cells = self._cells_in_rectangle(x - radius, y - radius, x + radius, y + radius)
        if cells is None:
            return None
        grid = self.terrain
        disk = []
        for row, col in cells:
            x0, y0 = grid.x_min + col * grid.resolution, grid.y_min + row * grid.resolution
            dx = max(x0 - x, 0, x - (x0 + grid.resolution))
            dy = max(y0 - y, 0, y - (y0 + grid.resolution))
            if hypot(dx, dy) <= radius:
                disk.append((row, col))
        low = max(self.lower[row][col] for row, col in disk)
        high = min(self.upper[row][col] for row, col in disk)
        return (low, high) if low < high else None

    def validate_loiter(self, circle: LoiterCircle) -> bool:
        band = self.loiter_band(circle.x, circle.y, circle.radius)
        if band is None or not band[0] < circle.z < band[1]:
            return False
        for box in self.obstacles:
            margin = self.obstacle_margin
            dx = max(box.x_min - margin - circle.x, 0, circle.x - box.x_max - margin)
            dy = max(box.y_min - margin - circle.y, 0, circle.y - box.y_max - margin)
            if (hypot(dx, dy) <= circle.radius
                    and box.z_min - margin <= circle.z <= box.z_max + margin):
                return False
        return True

    def choose_loiter(self, x, y, radius, altitude=None) -> LoiterCircle:
        """Equation (9): centre altitude is the midpoint of the safe disk band."""
        band = self.loiter_band(x, y, radius)
        if band is None:
            raise ValueError("no safe loiter band at this centre; change centre/radius/map")
        circle = LoiterCircle(x, y, sum(band) / 2 if altitude is None else altitude, radius)
        if not self.validate_loiter(circle):
            raise ValueError("loiter disk collides with terrain, altitude limits or an obstacle")
        return circle

    def path_is_free(self, path: DubinsAirplanePath, max_step=10.0, min_step=0.05) -> bool:
        """Conservative continuous-curve check, NOT just point sampling.

        A midpoint +/- half the XY arc length bounds the entire XY subcurve;
        altitude extrema are its endpoints because gamma is constant. Accept
        only if this box is free in every overlapping map cell. Otherwise
        subdivide; reject unresolved intervals at min_step (false negatives
        are possible). Even a thin obstacle between samples is not skipped.
        """
        if (not all(isfinite(v) for v in (max_step, min_step))
                or not 0 < min_step <= max_step):
            raise ValueError("require 0 < min_step <= max_step")
        if not self.is_free(path.start) or not self.is_free(path.goal):
            return False

        def interval(a, b):
            midpoint = path.pose_at((a + b) / 2)
            if not self.is_free(midpoint):
                return False
            half = (b - a) / 2
            z0, z1 = sorted((path.pose_at(a).z, path.pose_at(b).z))
            x0, y0, x1, y1 = midpoint.x - half, midpoint.y - half, midpoint.x + half, midpoint.y + half
            cells = self._cells_in_rectangle(x0, y0, x1, y1)
            box_free = (cells is not None
                        and all(self.lower[row][col] < z0 and z1 < self.upper[row][col]
                                for row, col in cells)
                        and not any(box.intersects(x0, y0, z0, x1, y1, z1, self.obstacle_margin)
                                    for box in self.obstacles))
            if box_free:
                return True
            if b - a <= min_step:
                return False
            middle = (a + b) / 2
            return interval(a, middle) and interval(middle, b)

        total = path.horizontal_length
        count = max(1, ceil(total / max_step))
        return all(interval(min(total, total * i / count),
                            min(total, total * (i + 1) / count)) for i in range(count))
