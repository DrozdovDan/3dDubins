"""Labeled 3D, top-down, and altitude views of Dubins-airplane routes.

Matplotlib is imported only when a visualization is requested. Each picture
answers three questions: where does the aircraft fly, which way does it turn,
and how does altitude change along the route? A GIF moves by path distance,
not by time; this planner has no speed model.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, cos, sin
from pathlib import Path
from typing import Sequence

from .core import DubinsAirplanePath, Pose


@dataclass(frozen=True)
class RouteLeg:
    label: str
    path: DubinsAirplanePath


@dataclass(frozen=True)
class Waypoint:
    label: str
    pose: Pose


def visualize_routes(
    legs: Sequence[RouteLeg],
    waypoints: Sequence[Waypoint],
    title: str,
    *,
    png: Path | None = None,
    gif: Path | None = None,
    show: bool = False,
    compare_alternatives: bool = False,
) -> None:
    """Save a labeled overview/GIF or display an interactive Matplotlib figure.

    With compare_alternatives=True, the distance axis of each leg starts at 0.
    Otherwise legs are consecutive and their distances are cumulative.
    """
    if not legs:
        raise ValueError("at least one route leg is required")
    if not (png or gif or show):
        return

    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation, PillowWriter
        from matplotlib.ticker import MaxNLocator
    except ImportError as exc:
        raise SystemExit("Install plotting packages: python -m pip install -r requirements.txt") from exc

    colors = ("#1769aa", "#dc6a24", "#25845b", "#8056a8")
    # Uniform horizontal-distance samples make the altitude-profile distance
    # axis exact even when the route has segments of very different lengths.
    sampled = []
    for leg in legs:
        count = max(1, ceil(leg.path.horizontal_length / 8.0))
        sampled.append([leg.path.pose_at(leg.path.horizontal_length * i / count)
                        for i in range(count + 1)])
    offsets = []
    distance = 0.0
    for leg in legs:
        offsets.append(0.0 if compare_alternatives else distance)
        distance += leg.path.length

    all_poses = [point for route in sampled for point in route]
    all_poses.extend(waypoint.pose for waypoint in waypoints)
    xs, ys, zs = ([getattr(point, coordinate) for point in all_poses]
                  for coordinate in ("x", "y", "z"))

    def bounds(values):
        low, high = min(values), max(values)
        pad = max(10.0, (high - low) * 0.08)
        return low - pad, high + pad

    xlim, ylim, zlim = bounds(xs), bounds(ys), bounds(zs)

    def make_figure(animated):
        figure = plt.figure(figsize=(12, 7.2))
        grid = figure.add_gridspec(2, 2, width_ratios=(1.18, 1))
        spatial = figure.add_subplot(grid[:, 0], projection="3d")
        map_view = figure.add_subplot(grid[0, 1])
        altitude = figure.add_subplot(grid[1, 1])
        figure.subplots_adjust(left=0.055, right=0.97, top=0.86,
                               bottom=0.17, wspace=0.27, hspace=0.37)
        figure.suptitle(title, fontsize=16, y=0.97)
        figure.text(0.5, 0.91,
                    "East / North in metres · arrows show travel direction · "
                    "GIF progress is by distance, not time",
                    ha="center", fontsize=9)

        flat_altitude = max(zs) - min(zs) < 1e-8
        horizontal_span = max(xlim[1] - xlim[0], ylim[1] - ylim[0])
        height_expanded = zlim[1] - zlim[0] < 0.25 * horizontal_span
        spatial.set(title="3D route (height expanded)" if height_expanded else "3D route",
                    xlabel="East (m)",
                    ylabel="North (m)", zlabel="Altitude (m)",
                    xlim=xlim, ylim=ylim, zlim=zlim)
        spatial.view_init(elev=24, azim=-62)
        spatial.set_box_aspect((xlim[1] - xlim[0], ylim[1] - ylim[0],
                                max(zlim[1] - zlim[0], 0.25 * horizontal_span)))
        spatial.xaxis.set_major_locator(MaxNLocator(nbins=4))
        spatial.yaxis.set_major_locator(MaxNLocator(nbins=4))
        if flat_altitude:
            spatial.set_zticks([zs[0]])
        else:
            spatial.zaxis.set_major_locator(MaxNLocator(nbins=4))
        map_view.set(title="Top-down: where and which way", xlabel="East (m)",
                     ylabel="North (m)", xlim=xlim, ylim=ylim)
        map_view.set_aspect("equal", adjustable="box")
        altitude.set(title="Altitude along the route", xlabel="Path distance (m)",
                     ylabel="Altitude (m)", ylim=zlim)
        for axis in (map_view, altitude):
            axis.grid(alpha=0.25)

        dynamic_lines = []
        for index, (leg, poses, offset) in enumerate(zip(legs, sampled, offsets)):
            color = colors[index % len(colors)]
            x = [point.x for point in poses]
            y = [point.y for point in poses]
            z = [point.z for point in poses]
            # Flight-path angle is constant within a leg, so sample index maps
            # linearly to its distance along that leg.
            d = [offset + leg.path.length * i / (len(poses) - 1)
                 for i in range(len(poses))] if len(poses) > 1 else [offset]
            opacity = 0.26 if animated else 0.95
            spatial.plot(x, y, z, color=color, lw=2.2, alpha=opacity)
            map_view.plot(x, y, color=color, lw=2.2, alpha=opacity,
                          label=(f"{leg.label}: {leg.path.word}, "
                                 f"{leg.path.altitude_case}, R={leg.path.radius:.0f} m, "
                                 f"{leg.path.extra_turns} turns, "
                                 f"{leg.path.length:.0f} m"))
            altitude.plot(d, z, color=color, lw=2.2, alpha=opacity,
                          # Medium/high share gamma_max: leave gaps in the
                          # later curve so the earlier curve remains visible.
                          ls="--" if compare_alternatives and index > 1 else "-")

            if len(poses) > 3:
                start, end = poses[len(poses) // 2], poses[min(len(poses) - 1,
                                                                  len(poses) // 2 + 4)]
                map_view.annotate("", xy=(end.x, end.y), xytext=(start.x, start.y),
                                  arrowprops=dict(arrowstyle="-|>", color=color,
                                                  lw=2, mutation_scale=14))
            # This arrow marks the required direction at the end of the leg.
            goal = leg.path.goal
            arrow_length = 0.05 * max(xlim[1] - xlim[0], ylim[1] - ylim[0])
            map_view.annotate("", xy=(goal.x + arrow_length * cos(goal.yaw),
                                       goal.y + arrow_length * sin(goal.yaw)),
                              xytext=(goal.x, goal.y),
                              arrowprops=dict(arrowstyle="->", color=color, lw=1.5))

            if animated:
                line3d, = spatial.plot([], [], [], color=color, lw=3.0)
                line2d, = map_view.plot([], [], color=color, lw=3.0)
                linealt, = altitude.plot([], [], color=color, lw=3.0)
                dynamic_lines.append((line3d, line2d, linealt, x, y, z, d))

        label_occurrences = {}
        location_counts = {}
        for waypoint in waypoints:
            key = (round(waypoint.pose.x, 8), round(waypoint.pose.y, 8))
            location_counts[key] = location_counts.get(key, 0) + 1
        for index, waypoint in enumerate(waypoints):
            point = waypoint.pose
            key = (round(point.x, 8), round(point.y, 8))
            occurrence = label_occurrences.get(key, 0)
            label_occurrences[key] = occurrence + 1
            marker = "o" if index == 0 else "s" if index == len(waypoints) - 1 else "D"
            face = "#287d51" if index == 0 else "#af3443" if index == len(waypoints) - 1 else "#ffffff"
            marker_size = 95 if occurrence == 0 and location_counts[key] > 1 else 58
            if occurrence and location_counts[key] > 1:
                marker_size = 36
            map_view.scatter(point.x, point.y, s=marker_size, marker=marker, c=face,
                             edgecolors="#222222", zorder=5)
            spatial.scatter(point.x, point.y, point.z, s=42, marker=marker,
                            c=face, edgecolors="#222222", depthshade=False)
            near_right = point.x > xlim[0] + 0.8 * (xlim[1] - xlim[0])
            near_top = point.y > ylim[0] + 0.8 * (ylim[1] - ylim[0])
            label_x = -6 if near_right else 6
            label_y = -30 - 26 * occurrence if near_top else 8 + 26 * occurrence
            map_view.annotate(f"{index + 1}. {waypoint.label}\n{point.z:.0f} m",
                              (point.x, point.y), xytext=(label_x, label_y),
                              textcoords="offset points", fontsize=8,
                              ha="right" if near_right else "left",
                              bbox=dict(facecolor="white", edgecolor="none", alpha=0.75))

        handles, labels = map_view.get_legend_handles_labels()
        figure.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, 0.055),
                      ncol=min(2, len(labels)), fontsize=8, frameon=False)
        figure.text(0.5, 0.016,
                    "Geometric route only: no terrain, airspace, wind, vehicle dynamics, or landing checks.",
                    ha="center", fontsize=8)
        return figure, spatial, map_view, altitude, dynamic_lines

    if png or show:
        figure, *_ = make_figure(animated=False)
        if png:
            png.parent.mkdir(parents=True, exist_ok=True)
            figure.savefig(png, dpi=130)
        if show:
            plt.show()
        plt.close(figure)

    if gif:
        figure, spatial, map_view, altitude, lines = make_figure(animated=True)
        marker3d, = spatial.plot([], [], [], marker="o", markersize=9,
                                 color="#bb233b", linestyle="None")
        marker_map, = map_view.plot([], [], marker="o", markersize=8,
                                    color="#bb233b", linestyle="None")
        marker_alt, = altitude.plot([], [], marker="o", markersize=8,
                                    color="#bb233b", linestyle="None")
        total = sum(leg.path.length for leg in legs)
        frame_count = 55

        def update(frame):
            progress = total * frame / frame_count
            done = 0.0
            active = len(legs) - 1
            fraction = 1.0
            for index, leg in enumerate(legs):
                if progress <= done + leg.path.length or index == len(legs) - 1:
                    active = index
                    fraction = (progress - done) / leg.path.length if leg.path.length else 0.0
                    fraction = min(1.0, max(0.0, fraction))
                    break
                done += leg.path.length

            for index, (line3d, line2d, linealt, x, y, z, d) in enumerate(lines):
                if index < active:
                    count = len(x)
                elif index == active:
                    count = max(1, int(fraction * (len(x) - 1)) + 1)
                else:
                    count = 0
                line3d.set_data_3d(x[:count], y[:count], z[:count])
                line2d.set_data(x[:count], y[:count])
                linealt.set_data(d[:count], z[:count])

            leg = legs[active]
            pose = leg.path.pose_at(fraction * leg.path.horizontal_length)
            marker3d.set_data_3d([pose.x], [pose.y], [pose.z])
            marker_map.set_data([pose.x], [pose.y])
            marker_alt.set_data([offsets[active] + fraction * leg.path.length], [pose.z])
            figure.suptitle(f"{title} — showing {leg.label}", fontsize=16, y=0.97)
            return (*[line for group in lines for line in group[:3]],
                    marker3d, marker_map, marker_alt)

        animation = FuncAnimation(figure, update, frames=frame_count + 1,
                                  interval=100, blit=False, repeat=True)
        gif.parent.mkdir(parents=True, exist_ok=True)
        animation.save(gif, writer=PillowWriter(fps=10), dpi=80)
        plt.close(figure)
