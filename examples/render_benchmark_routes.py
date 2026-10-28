"""Actual Rerun oblique screenshots of recorded benchmark paths; never replans.

python -m examples.render_benchmark_routes outputs/benchmark_swarm_1km_bidirectional/results.json
Requires rerun_scripts/requirements.txt and Pillow. Saves editable .rrd scenes,
full-resolution per-scene PNGs and a slide-ready 16:9 routes.png contact sheet.
"""

from __future__ import annotations

import argparse
import json
from math import cos, radians, sin
from pathlib import Path
import socket
import time

from .benchmark_scenarios import DIRECTIONS, SCENARIOS, TOPOLOGIES, describe_scene, goal_region, make_scene

COLORS = [(0, 102, 215), (229, 92, 0), (0, 151, 112), (174, 57, 188),
          (218, 42, 81), (0, 157, 186), (151, 109, 0), (85, 78, 194)]


def select_runs(report, direction=None):
    """Fixed representatives, NOT a search for whichever tasks succeeded."""
    chosen = []
    directions = [d for d in DIRECTIONS if any(r.get('direction', 'ascent') == d for r in report['runs'])]
    if not directions or (direction is not None and direction not in directions):
        raise ValueError('no benchmark tasks for the requested swarm direction')
    for index, topology in enumerate(TOPOLOGIES):
        selected = [r for r in report['runs'] if r['topology'] == topology
                    and (direction is None or r.get('direction', 'ascent') == direction)]
        if direction is None:
            preferred = ('descent', 'ascent')[index % 2]
            if preferred not in directions:
                preferred = directions[0]
            selected = [r for r in selected if r.get('direction', 'ascent') == preferred] or selected
        candidates = [r for r in selected if r['scenario'] == 'reconfigure']
        if not candidates:
            candidates = selected
        if candidates:
            chosen.append(next((r for r in candidates if r['agents'] == 5), candidates[-1]))
    if not chosen:
        raise ValueError('no benchmark tasks to render')
    return chosen


def record_scene(run, scene, output, *, elevation=55, azimuth=-135):
    import rerun as rr
    import rerun.blueprint as rrb
    from rerun_scripts.dubins import terrain_mesh, terrain_risers

    world = make_scene(run['topology'], run['scenario'])
    direction = run.get('direction', 'ascent')
    current = describe_scene(world, run['topology'], run['scenario'], direction)
    for key in ('obstacles', 'overview_heights', 'extent_m', 'cell_m', 'flight_bounds_m'):
        if current[key] != scene[key]:
            raise ValueError(f'{key} changed since measurement; refusing to draw a different map')
    rr.init(f"benchmark-{run['id'].replace('/', '-')}", spawn=False, strict=True)
    rr.save(str(output))
    target, distance = [500, 0, 575], 2050
    el, az = radians(elevation), radians(azimuth)
    eye = [target[0]+distance*cos(el)*cos(az), target[1]+distance*cos(el)*sin(az),
           target[2]+distance*sin(el)]
    rr.send_blueprint(rrb.Blueprint(
        rrb.Spatial3DView(origin='/world', name=run['id'], background=[242, 245, 249],
                          eye_controls=rrb.EyeControls3D(position=eye, look_target=target, eye_up=[0, 0, 1]),
                          line_grid=False),
        rrb.TimePanel(state='hidden'), collapse_panels=True))
    rr.log('world', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    for obj in (terrain_mesh(world.terrain), terrain_risers(world.terrain)):
        rr.log(f"world/{obj['name']}", rr.Mesh3D(vertex_positions=obj['vertices'],
               triangle_indices=obj['faces'], vertex_colors=obj['vertex_colors']), static=True)
    for i, box in enumerate(world.obstacles):
        size = [box.x_max-box.x_min, box.y_max-box.y_min, box.z_max-box.z_min]
        center = [(box.x_max+box.x_min)/2, (box.y_max+box.y_min)/2, (box.z_max+box.z_min)/2]
        rr.log(f'world/obstacles/{i}/fill', rr.Boxes3D(centers=[center], sizes=[size],
               colors=[145, 100, 85, 35], fill_mode='solid'), static=True)
        rr.log(f'world/obstacles/{i}/outline', rr.Boxes3D(centers=[center], sizes=[size],
               colors=[135, 108, 98, 130], fill_mode=rr.components.FillMode.MajorWireframe,
               radii=rr.Radius.ui_points(.7)), static=True)
    # Actual workspace, equal physical scales on all axes. No vertical exaggeration.
    rr.log('world/bounds', rr.Boxes3D(mins=[[0, -500, 150]], sizes=[[1000, 1000, 1000]],
           colors=[70, 85, 105, 90], fill_mode=rr.components.FillMode.MajorWireframe,
           radii=rr.Radius.ui_points(.7)), static=True)
    rr.log('world/scale', rr.Points3D([[500, -500, 150], [0, 0, 150], [0, -500, 650]],
           colors=[70, 85, 105], radii=rr.Radius.ui_points(0),
           labels=['x: 1000 м', 'y: 1000 м', 'Δz: 1000 м'], show_labels=True), static=True)
    region = scene['mission']['goal_region_m']
    if region != goal_region(direction):
        raise ValueError('recorded destination region does not match the swarm direction')
    x0, x1 = region['x']
    y0, y1 = region['y']
    z = region['z'][0]
    rr.log('world/goal_region', rr.LineStrips3D([[[x0, y0, z], [x1, y0, z],
           [x1, y1, z], [x0, y1, z], [x0, y0, z]]],
           colors=[15, 115, 160], radii=rr.Radius.ui_points(1.5)), static=True)
    for route in run['final_routes']:
        name, points = route['agent'], route['points']
        color = COLORS[(ord(name)-65) % len(COLORS)]
        rr.log(f'world/routes/{name}', rr.LineStrips3D([points], colors=color,
               radii=rr.Radius.ui_points(2.7)), static=True)
        # Short arrows lie along the recorded path, not a fictitious straight
        # start-goal segment: descent must be visible without reading tiny tags.
        indices = sorted({min(len(points)-2, int(f*(len(points)-1))) for f in (.2, .5, .8)})
        rr.log(f'world/direction/{name}', rr.Arrows3D(
               origins=[points[k] for k in indices],
               vectors=[[b-a for a, b in zip(points[k], points[min(k+2, len(points)-1)])] for k in indices],
               colors=color, radii=rr.Radius.ui_points(1.7)), static=True)
        rr.log(f'world/endpoints/{name}', rr.Points3D([points[0], points[-1]],
               colors=color, radii=rr.Radius.ui_points(5), labels=[f'{name} старт', f'{name} цель'],
               show_labels=True), static=True)
    rr.get_global_data_recording().flush()
    rr.disconnect()


def capture(recording, output):
    """Screenshot the 3D viewport itself, not the UI and not a Matplotlib stand-in."""
    from rerun.experimental import ViewerClient
    from rerun_scripts.rerunlib import viewer_executable

    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    with ViewerClient.spawn(headless=True, port=port, hide_welcome_screen=True,
                            executable_path=viewer_executable()) as client:
        client.open_url(str(recording.resolve()))
        deadline = time.monotonic()+40
        while time.monotonic() < deadline:
            state = client.viewer_state()
            if state.active_recording and len(state.views) == 1:
                break
            time.sleep(.1)
        else:
            raise TimeoutError(f'Rerun did not load {recording}')
        # Geometry uploads and GPU rendering finish after blueprint loading.
        time.sleep(2)
        view = state.views[0]
        client.save_screenshot(str(output.resolve()), view_id=view.view_id)
        reports = [(v.name, r.summary) for v in client.viewer_state().views for r in v.reports]
        if reports:
            raise RuntimeError(f'Rerun visualization reports: {reports}')


def compose(chosen, images, output):
    from PIL import Image, ImageChops, ImageDraw, ImageFont, ImageOps

    def font(size, bold=False):
        for candidate in (Path('C:/Windows/Fonts')/('arialbd.ttf' if bold else 'arial.ttf'),
                          Path('/usr/share/fonts/truetype/dejavu')/('DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf')):
            if candidate.is_file():
                return ImageFont.truetype(str(candidate), size)
        return ImageFont.load_default(size=size)

    canvas = Image.new('RGB', (3840, 2160), '#f2f5f9')
    draw = ImageDraw.Draw(canvas)
    draw.text((90, 45), 'Совместные маршруты группы · Rerun 3D', font=font(76, True), fill='#16273b')
    scenarios = {r['scenario'] for r in chosen}
    mission = SCENARIOS[next(iter(scenarios))] if len(scenarios) == 1 else 'Групповые миссии'
    directions = {r.get('direction', 'ascent') for r in chosen}
    movement = DIRECTIONS[next(iter(directions))] if len(directions) == 1 else 'Подъём и спуск роя'
    counts = ', '.join(str(n) for n in sorted({r['agents'] for r in chosen}))
    draw.text((90, 150), f'{mission} · {movement} · агентов: {counts} · 1 × 1 × 1 км · перепад 1000 м',
              font=font(40), fill='#43566d')
    for i, (run, image_path) in enumerate(zip(chosen, images)):
        x, y = 65+(i % 3)*1250, 265+(i // 3)*865
        draw.text((x+22, y), f"{i+1}. {TOPOLOGIES[run['topology']]}", font=font(43, True), fill='#16273b')
        direction = run.get('direction', 'ascent')
        status = (f"{DIRECTIONS[direction]} {'+1000' if direction == 'ascent' else '−1000'} м · "
                  f"согласовано {len(run['final_routes'])}/{run['agents']}" if run['complete'] else 'НЕПОЛНАЯ МИССИЯ')
        draw.text((x+22, y+62), status, font=font(30), fill='#43566d' if run['complete'] else '#b02035')
        with Image.open(image_path) as source:
            source = source.convert('RGB')
            # Trim only empty viewport margins, not geometry; keep raw captures
            # alongside the collage. Never stretch the scene or its altitude.
            difference = ImageChops.difference(source, Image.new('RGB', source.size, source.getpixel((0, 0))))
            box = difference.convert('L').point(lambda p: 255 if p > 8 else 0).getbbox()
            if box:
                source = source.crop((max(0, box[0]-30), max(0, box[1]-30),
                                      min(source.width, box[2]+30), min(source.height, box[3]+30)))
            panel = ImageOps.contain(source, (1230, 735), Image.Resampling.LANCZOS)
            canvas.paste(panel, (x+(1230-panel.width)//2, y+108+(735-panel.height)//2))
    y = 2020
    for i, color in enumerate(COLORS[:max(r['agents'] for r in chosen)]):
        x = 90+i*190
        draw.line((x, y+20, x+65, y+20), fill=color, width=10)
        draw.text((x+80, y-6), chr(65+i), font=font(40, True), fill='#16273b')
    draw.text((1680, y-6), 'Линии — пути; стрелки — полёт; рамка — общая цель всего роя',
              font=font(31), fill='#43566d')
    draw.text((90, 2100), 'Вид сверху-сбоку · масштаб высоты не искажён · рельеф: клетки 10 × 10 м · препятствия полупрозрачны',
              font=font(32), fill='#43566d')
    canvas.save(output)


def render_routes(report, destination, *, elevation=55, azimuth=-135, compose_only=False):
    if not 5 <= elevation <= 85:
        raise ValueError('camera elevation must be within 5..85 degrees')
    chosen = select_runs(report)
    directions = [d for d in DIRECTIONS if any(r.get('direction', 'ascent') == d for r in report['runs'])]
    variants = {d: select_runs(report, d) for d in directions}
    all_runs = {r['id']: r for selected in variants.values() for r in selected}
    destination = Path(destination)
    folder = destination/'rerun'
    folder.mkdir(parents=True, exist_ok=True)
    if compose_only:
        metadata = json.loads((destination/'routes.render.json').read_text(encoding='utf-8'))
        if (metadata.get('renderer') != 'Rerun Viewer' or metadata.get('camera_elevation_deg') != elevation
                or metadata.get('camera_azimuth_deg') != azimuth):
            raise ValueError('cached Rerun camera differs; render new views without --compose-only')
    images = {}
    for run in all_runs.values():
        stem = run['id'].replace('/', '_')
        rrd, png = folder/f'{stem}.rrd', folder/f'{stem}.png'
        if compose_only:
            relative = metadata.get('source_images', {}).get(run['id'])
            if relative is None or (destination/relative).resolve() != png.resolve() or not png.is_file():
                raise ValueError(f'missing recorded Rerun image for {run["id"]}; render without --compose-only')
        else:
            record_scene(run, report['scenes'][run['scene_id']], rrd, elevation=elevation, azimuth=azimuth)
            capture(rrd, png)
        images[run['id']] = png
        print(f'{"Reusing Rerun" if compose_only else "Rerun"}: {run["id"]} -> {png}', flush=True)
    compose(chosen, [images[r['id']] for r in chosen], destination/'routes.png')
    for d, selected in variants.items():
        compose(selected, [images[r['id']] for r in selected], destination/f'routes_{d}.png')
    (destination/'routes.render.json').write_text(json.dumps(dict(
        renderer='Rerun Viewer', camera_elevation_deg=elevation, camera_azimuth_deg=azimuth,
        vertical_exaggeration=1, terrain_cell_m=10, tasks=[r['id'] for r in chosen],
        variants={d: [r['id'] for r in selected] for d, selected in variants.items()},
        source_images={name: str(p.relative_to(destination)) for name, p in images.items()}), indent=2), encoding='utf-8')
    return destination/'routes.png'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results', type=Path)
    parser.add_argument('--camera-elevation', type=float, default=55)
    parser.add_argument('--camera-azimuth', type=float, default=-135)
    parser.add_argument('--compose-only', action='store_true',
                        help='only combine existing Rerun PNGs; no Viewer or planning')
    args = parser.parse_args()
    report = json.loads(args.results.read_text(encoding='utf-8'))
    print(render_routes(report, args.results.parent/'report', elevation=args.camera_elevation,
                        azimuth=args.camera_azimuth, compose_only=args.compose_only))


if __name__ == '__main__':
    main()
