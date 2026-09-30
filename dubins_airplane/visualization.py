"""Labeled 3D, top-down, and altitude views of Dubins-airplane routes.

Matplotlib is imported only when a visualization is requested. Each picture
answers three questions: where does the aircraft fly, which way does it turn,
and how does altitude change along the route? A GIF moves by path distance,
not by time; this planner has no speed model.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, cos, degrees, sin
from pathlib import Path
from typing import TYPE_CHECKING, Sequence

from .core import DubinsAirplanePath, Pose

if TYPE_CHECKING:
    from .terrain import FlightWorld, LoiterCircle


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
    world: FlightWorld | None = None,
    loiters: Sequence[LoiterCircle] = (),
    rejected_path: DubinsAirplanePath | None = None,
    reference_path: DubinsAirplanePath | None = None,
    description: str | None = None,
) -> None:
    """Save a labeled overview/GIF or display an interactive Matplotlib figure.

    With compare_alternatives=True, the distance axis of each leg starts at 0.
    Otherwise legs are consecutive and their distances are cumulative.
    With a world, use a map-first Russian overview with one route color and
    an altitude corridor. reference_path is classified by the continuous
    collision checker; rejected_path is its backward-compatible alias.
    """
    if not legs:
        raise ValueError("at least one route leg is required")
    if rejected_path is not None and reference_path is not None:
        raise ValueError("provide reference_path or rejected_path, not both")
    if not (png or gif or show):
        return
    if world is not None:
        if compare_alternatives:
            raise ValueError("world visualization requires consecutive route legs")
        _visualize_world(legs, waypoints, title, world, loiters,
                         reference_path if reference_path is not None else rejected_path,
                         description, png, gif, show)
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
        sampled.append([leg.path.pose_at(min(leg.path.horizontal_length,
                                            leg.path.horizontal_length * i / count))
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
    spatial_zlim = zlim

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
        height_expanded = spatial_zlim[1] - spatial_zlim[0] < 0.25 * horizontal_span
        spatial.set(title="3D route (height expanded)" if height_expanded else "3D route",
                    xlabel="East (m)",
                    ylabel="North (m)", zlabel="Altitude (m)",
                    xlim=xlim, ylim=ylim, zlim=spatial_zlim)
        spatial.view_init(elev=24, azim=-62)
        spatial.set_box_aspect((xlim[1] - xlim[0], ylim[1] - ylim[0],
                                max(spatial_zlim[1] - spatial_zlim[0], 0.25 * horizontal_span)))
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
                count = len(x) if index < active else (
                    max(1, int(fraction * (len(x) - 1)) + 1) if index == active else 0)
                line3d.set_data_3d(x[:count], y[:count], z[:count])
                line2d.set_data(x[:count], y[:count])
                linealt.set_data(d[:count], z[:count])

            leg = legs[active]
            pose = leg.path.pose_at(min(leg.path.horizontal_length, fraction * leg.path.horizontal_length))
            marker3d.set_data_3d([pose.x], [pose.y], [pose.z])
            marker_map.set_data([pose.x], [pose.y])
            marker_alt.set_data([offsets[active] + fraction * leg.path.length], [pose.z])
            figure.suptitle(f"{title} — showing {leg.label}", fontsize=16, y=0.97)
            return (*[line for group in lines for line in group[:3]], marker3d, marker_map, marker_alt)

        animation = FuncAnimation(figure, update, frames=frame_count + 1,
                                  interval=100, blit=False, repeat=True)
        gif.parent.mkdir(parents=True, exist_ok=True)
        animation.save(gif, writer=PillowWriter(fps=10), dpi=80)
        plt.close(figure)


def _route_samples(legs: Sequence[RouteLeg], step: float = 5.0):
    """Return one continuous polyline, exact 3D distances and leg offsets."""
    poses, distances, offsets = [], [], []
    done = 0.0
    for index, leg in enumerate(legs):
        offsets.append(done)
        count = max(1, ceil(leg.path.horizontal_length / step))
        for i in range(1 if index else 0, count + 1):
            poses.append(leg.path.pose_at(min(leg.path.horizontal_length,
                                              leg.path.horizontal_length * i / count)))
            distances.append(done + leg.path.length * i / count)
        done += leg.path.length
    return poses, distances, offsets


def _visualize_world(legs, waypoints, title, world, loiters, reference,
                     description, png, gif, show):
    """A map-first view: one route color and an explicit height corridor.

    Depth sorting is disabled deliberately: a transparent terrain mesh must
    not hide the route. Continuous safety is checked by FlightWorld, not by
    the visual point samples or the artificial rendering order.
    """
    try:
        import matplotlib.pyplot as plt
        import numpy as np
        from matplotlib.animation import FuncAnimation, PillowWriter
        from matplotlib.lines import Line2D
        from matplotlib.patches import Patch, Rectangle
        from matplotlib.ticker import MaxNLocator
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    except ImportError as exc:
        raise SystemExit("Install plotting packages: python -m pip install -r requirements.txt") from exc

    blue, teal, red = "#1667b1", "#008b83", "#c03935"
    poses, distances, offsets = _route_samples(legs)
    x, y, z = (np.array([getattr(p, key) for p in poses]) for key in ("x", "y", "z"))
    bands = [world.altitude_band(p.x, p.y) for p in poses]
    lower = np.array([b[0] if b else np.nan for b in bands])
    upper = np.array([b[1] if b else np.nan for b in bands])
    ground = np.array([world.terrain.height_at(p.x, p.y) for p in poses])
    ref = reference.sample(5) if reference is not None else []
    ref_free = world.path_is_free(reference) if reference is not None else True
    ref_color = "#777777" if ref_free else red
    ref_label = "Прямой путь: свободен" if ref_free else "Прямой путь: заблокирован"
    grid_map = world.terrain
    terrain = np.array(grid_map.heights)
    gx = grid_map.x_min + (np.arange(grid_map.columns) + 0.5) * grid_map.resolution
    gy = grid_map.y_min + (np.arange(grid_map.rows) + 0.5) * grid_map.resolution
    xx, yy = np.meshgrid(gx, gy)
    bounds_points = list(poses) + ref + [w.pose for w in waypoints]
    for circle in loiters:
        bounds_points.extend(circle.states(32))
    bx = [p.x for p in bounds_points] + [v for b in world.obstacles for v in (b.x_min, b.x_max)]
    by = [p.y for p in bounds_points] + [v for b in world.obstacles for v in (b.y_min, b.y_max)]

    def limits(values, minimum=None, maximum=None, padding=25.0):
        low, high = min(values), max(values)
        pad = max(padding, (high - low) * 0.10)
        return (max(low - pad, minimum) if minimum is not None else low - pad,
                min(high + pad, maximum) if maximum is not None else high + pad)

    xlim = limits(bx, grid_map.x_min, grid_map.x_max)
    ylim = limits(by, grid_map.y_min, grid_map.y_max, padding=65)
    # Matplotlib's 3D surfaces do not clip like imshow. Restrict the mesh to
    # the displayed region so distant map cells do not spill outside the axes.
    visible_columns = np.flatnonzero((gx >= xlim[0]) & (gx <= xlim[1]))
    visible_rows = np.flatnonzero((gy >= ylim[0]) & (gy <= ylim[1]))
    visible = np.ix_(visible_rows, visible_columns)
    zlim = limits([*z, float(terrain.min()), float(terrain.max()),
                   *[b.z_max for b in world.obstacles]])
    profile_lim = limits([*z, *ground, *lower[np.isfinite(lower)], *upper[np.isfinite(upper)]])
    total = sum(leg.path.length for leg in legs)
    legend = [Line2D([], [], color=blue, lw=3, label="Найденный маршрут")]
    if ref:
        legend.append(Line2D([], [], color=ref_color, lw=2, ls="--", label=ref_label))
    legend.extend((Line2D([], [], color=teal, lw=2, ls="--", label="Круги ожидания"),
                   Patch(facecolor="#d8eddd", edgecolor="#699576", label="Допустимая высота")))
    if world.obstacles:
        legend.append(Patch(facecolor="#f6d7aa", edgecolor="#b1742e", hatch="///",
                            label="Запретный объём"))

    def make_figure(animated=False):
        figure = plt.figure(figsize=(14, 8.5))
        layout = figure.add_gridspec(2, 2, height_ratios=(1.35, 1), width_ratios=(1.1, 1))
        map_view = figure.add_subplot(layout[0, 0])
        spatial = figure.add_subplot(layout[0, 1], projection="3d", computed_zorder=False)
        altitude = figure.add_subplot(layout[1, :])
        figure.subplots_adjust(left=0.07, right=0.95, bottom=0.18, top=0.84,
                               wspace=0.18, hspace=0.42)
        figure.suptitle(title, fontsize=17, y=0.98)
        if description:
            figure.text(0.5, 0.937, description, ha="center", fontsize=10)
        figure.text(0.5, 0.9,
                    f"Длина: {total:.0f} м  ·  Участков: {len(legs)}  ·  "
                    f"Радиус ≥ {legs[0].path.minimum_radius:.0f} м  ·  "
                    f"Наклон ≤ {degrees(legs[0].path.max_flight_path_angle):.0f}°",
                    ha="center", fontsize=10)
        map_view.set(title="1. Где пролетаем — вид сверху", xlabel="Восток, м",
                     ylabel="Север, м", xlim=xlim, ylim=ylim)
        map_view.set_aspect("equal", adjustable="box")
        spatial.set(title="2. Маршрут в 3D", xlabel="Восток, м", ylabel="Север, м",
                    zlabel="Высота, м", xlim=xlim, ylim=ylim, zlim=zlim)
        spatial.view_init(elev=27, azim=-65)
        spatial.set_box_aspect((xlim[1] - xlim[0], ylim[1] - ylim[0],
                                max(zlim[1] - zlim[0], 0.28 * (xlim[1] - xlim[0]))))
        for axis in (spatial.xaxis, spatial.yaxis, spatial.zaxis):
            axis.set_major_locator(MaxNLocator(4))
            axis.set_tick_params(labelsize=8)
        altitude.set(title="3. Высота вдоль найденного маршрута", xlabel="Расстояние по маршруту, м",
                     ylabel="Высота, м", xlim=(-total * 0.015, max(1, total * 1.015)), ylim=profile_lim)
        map_view.grid(alpha=0.15)
        altitude.grid(alpha=0.2)
        map_view.imshow(terrain, extent=(grid_map.x_min, grid_map.x_max,
                                         grid_map.y_min, grid_map.y_max), origin="lower",
                        cmap="Greys", alpha=0.28, vmin=0, vmax=max(1, float(terrain.max())))
        if terrain.max() - terrain.min() > 1:
            contours = map_view.contour(xx, yy, terrain, levels=4, colors="#87877e",
                                        linewidths=0.7, alpha=0.85)
            map_view.clabel(contours, fontsize=8, fmt=lambda h: f"{h:.0f} м")
        spatial.plot_surface(xx[visible], yy[visible], terrain[visible], color="#9c9b87", alpha=0.22,
                             linewidth=0, antialiased=False, zorder=0)
        for index, box in enumerate(world.obstacles, 1):
            map_view.add_patch(Rectangle((box.x_min, box.y_min), box.x_max - box.x_min,
                                         box.y_max - box.y_min, facecolor="#f6d7aa",
                                         edgecolor="#b1742e", hatch="///", lw=1, zorder=2))
            map_view.text((box.x_min + box.x_max) / 2, (box.y_min + box.y_max) / 2,
                          f"П{index}\n{box.z_min:.0f}–{box.z_max:.0f} м", fontsize=8,
                          ha="center", va="center", zorder=7,
                          bbox=dict(facecolor="#fff7e9", edgecolor="none", alpha=0.9))
            vertices = [(a, b, c) for c in (box.z_min, box.z_max)
                        for b in (box.y_min, box.y_max) for a in (box.x_min, box.x_max)]
            faces = [[vertices[i] for i in ids] for ids in
                     ((0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4),
                      (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5))]
            spatial.add_collection3d(Poly3DCollection(faces, facecolor="#e8ad60",
                                                     edgecolor="#ac7a3f", alpha=0.18, zorder=1))
        for circle in loiters:
            states = circle.states(64)
            points = (*states, states[0])
            a, b, c = ([getattr(p, key) for p in points] for key in ("x", "y", "z"))
            map_view.plot(a, b, color=teal, ls="--", lw=1.8, zorder=3)
            spatial.plot(a, b, c, color=teal, ls="--", lw=1.8, zorder=3)
        if ref:
            a, b, c = ([getattr(p, key) for p in ref] for key in ("x", "y", "z"))
            map_view.plot(a, b, color=ref_color, ls="--", lw=1.7, zorder=4)
            spatial.plot(a, b, c, color=ref_color, ls="--", lw=1.7, zorder=4)
            # Diagnostic crosses locate blocked point samples; only the
            # continuous checker above decides whether the curve is free.
            bad = [p for p in ref if not world.is_free(p)]
            if bad:
                point = bad[len(bad) // 2]
                map_view.plot(point.x, point.y, "x", color=red, ms=10, mew=2, zorder=8)
                spatial.plot([point.x], [point.y], [point.z], "x", color=red, ms=9, zorder=8)
        altitude.fill_between(distances, profile_lim[0], ground, color="#aaa58c", alpha=0.35)
        altitude.plot(distances, ground, color="#817c65", lw=1, label="Рельеф под маршрутом")
        altitude.fill_between(distances, lower, upper, color="#d8eddd", alpha=0.9)
        altitude.plot(distances, lower, color="#699576", ls="--", lw=1)
        altitude.plot(distances, upper, color="#699576", ls="--", lw=1)
        altitude.legend(loc="upper left", fontsize=8, framealpha=0.85)
        opacity = 0.3 if animated else 1.0
        map_view.plot(x, y, color=blue, lw=2.8, alpha=opacity, zorder=5)
        spatial.plot(x, y, z, color=blue, lw=2.8, alpha=opacity, zorder=5)
        altitude.plot(distances, z, color=blue, lw=2.8, alpha=opacity, zorder=5)
        # Join numbers describe RRT* legs without suggesting separate routes.
        for index, offset in enumerate(offsets[1:], 1):
            point = legs[index].path.start
            map_view.plot(point.x, point.y, "o", ms=4, color=blue, zorder=6)
            map_view.annotate(str(index), (point.x, point.y), xytext=(0, 9),
                              textcoords="offset points", fontsize=8, ha="center", color=blue,
                              bbox=dict(facecolor="white", edgecolor="none", alpha=0.85), zorder=8)
            altitude.axvline(offset, color=blue, alpha=0.2, ls=":", lw=1)
        for index, point in enumerate((poses[0], poses[-1])):
            marker, color = ("o", "#25834b") if index == 0 else ("s", blue)
            label = "Старт" if index == 0 else "Финиш"
            map_view.plot(point.x, point.y, marker, color=color, mec="white", ms=8, zorder=9)
            spatial.plot([point.x], [point.y], [point.z], marker, color=color, ms=7, zorder=9)
            altitude.plot(distances[0 if index == 0 else -1], point.z, marker,
                          color=color, mec="white", ms=7, zorder=9)
            map_view.annotate(f"{label}\n{point.z:.0f} м", (point.x, point.y),
                              xytext=(0, 28), textcoords="offset points", fontsize=9,
                              ha="center", bbox=dict(facecolor="white", edgecolor="none", alpha=0.85),
                              arrowprops=dict(arrowstyle="-", color=color), zorder=10)
        # An arrow along the first leg makes start-to-finish direction explicit.
        i = min(len(poses) - 2, max(0, len(poses) // 5))
        j = min(len(poses) - 1, i + 5)
        map_view.annotate("", xy=(x[j], y[j]), xytext=(x[i], y[i]),
                          arrowprops=dict(arrowstyle="-|>", color=blue, lw=2), zorder=7)
        figure.legend(handles=legend, loc="lower center", bbox_to_anchor=(0.5, 0.065),
                      ncol=3, fontsize=9, frameon=False)
        figure.text(0.5, 0.035,
                    ("П1, П2…: объёмы с указанной высотой; " if world.obstacles else "")
                    + "Цифры на линии — стыки участков. "
                    "Зелёная полоса — коридор именно вдоль синего пути.", ha="center", fontsize=8)
        figure.text(0.5, 0.012,
                    "GIF: прогресс по расстоянию, не по времени. Проверена статическая карта; "
                    "ветер и ошибки управления не учтены.", ha="center", fontsize=8)
        return figure, map_view, spatial, altitude

    if png or show:
        figure, *_ = make_figure()
        if png:
            png.parent.mkdir(parents=True, exist_ok=True)
            figure.savefig(png, dpi=140)
        if show:
            plt.show()
        plt.close(figure)
    if gif:
        from bisect import bisect_right

        figure, map_view, spatial, altitude = make_figure(animated=True)
        trace_map, = map_view.plot([], [], color=blue, lw=3, zorder=6)
        trace_3d, = spatial.plot([], [], [], color=blue, lw=3, zorder=6)
        trace_alt, = altitude.plot([], [], color=blue, lw=3, zorder=6)
        marker_map, = map_view.plot([], [], "o", color=red, ms=7, zorder=11)
        marker_3d, = spatial.plot([], [], [], "o", color=red, ms=7, zorder=11)
        marker_alt, = altitude.plot([], [], "o", color=red, ms=7, zorder=11)
        status = figure.text(0.5, 0.862, "", ha="center", fontsize=9, color=blue)

        def update(frame):
            progress = total * frame / 55
            active = min(len(legs) - 1, max(0, bisect_right(offsets, progress) - 1))
            path = legs[active].path
            fraction = min(1.0, max(0.0, (progress - offsets[active]) / path.length)) if path.length else 1.0
            point = path.pose_at(min(path.horizontal_length, fraction * path.horizontal_length))
            count = bisect_right(distances, progress)
            a, b, c = [*x[:count], point.x], [*y[:count], point.y], [*z[:count], point.z]
            trace_map.set_data(a, b)
            trace_3d.set_data_3d(a, b, c)
            trace_alt.set_data([*distances[:count], progress], c)
            marker_map.set_data([point.x], [point.y])
            marker_3d.set_data_3d([point.x], [point.y], [point.z])
            marker_alt.set_data([progress], [point.z])
            status.set_text(f"Пройдено {progress:.0f} / {total:.0f} м  ·  Высота {point.z:.0f} м")
            return trace_map, trace_3d, trace_alt, marker_map, marker_3d, marker_alt, status

        animation = FuncAnimation(figure, update, frames=56, interval=100, blit=False)
        gif.parent.mkdir(parents=True, exist_ok=True)
        animation.save(gif, writer=PillowWriter(fps=10), dpi=100)
        plt.close(figure)
