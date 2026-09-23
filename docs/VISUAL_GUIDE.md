# Visual guide to the examples

The pictures show the same route in three ways. The **3D view** shows the actual shape; the **top-down view** shows where the aircraft flies and its travel direction; the **altitude profile** shows how high it is after a given distance along the path. Coordinates are local metres: `x` is east, `y` is north, and `z` is altitude upward. `yaw=0°` means east, `90°` north, and `180°` west.

The green circle is the first waypoint, a red square is the last, and white diamonds are intermediate waypoints. Numbered labels tell you the requested location and altitude. Each colored curve is one route leg. The small arrows point in the travel direction, including the requested heading at the end of a leg. GIFs move in equal **distance** increments, not equal time increments: this planner does not model speed.

Read each three-letter path code from start to finish: `L` is a left turn, `R` a right turn, and `S` a straight segment in the horizontal plane. For example, `RSR` means right turn → straight → right turn. The legend also gives the number of complete helical turns added for altitude and the total **3D** length of that leg. A climbing helix is placed before the planar path; a descending helix is placed after it.

For nearly level routes, the 3D panel expands the vertical display scale to keep it legible. Its altitude labels and the altitude profile retain the real metre values.

## 1. Two-leg route: depot → high transfer point → destination

The aircraft begins at the depot at 120 m heading east, climbs to the transfer point at 500 m heading north, then descends to the destination at 100 m heading west. The maximum climb/descent angle is 15° and the minimum horizontal turn radius is 60 m. Each leg adds one full helical turn to gain enough horizontal distance for its altitude change.

![Animation of two-leg climb and descent](../visuals/two_leg_route.gif)

[Open a still image](../visuals/two_leg_route.png) · Code: [`example.py`](../examples/example.py)

## 2. Same destination, different arrival direction

Both alternatives leave the same depot and reach `(450, 250, 120)`. One must arrive facing east and the other facing west. The west-facing route is longer because the aircraft cannot turn in place: it must obey the 60 m minimum turn radius. Both stay level, so their altitude profiles are flat.

![Animation comparing east- and west-facing arrivals](../visuals/arrival_headings.gif)

[Open a still image](../visuals/arrival_headings.png) · Code: [`example_arrival_heading.py`](../examples/example_arrival_heading.py)

## 3. Climb above one ground point

The target has the **same** `(x, y)` and heading as the start, but is 200 m higher. A forward-flying aircraft cannot go straight up, so the route circles three times while climbing. The altitude profile is a straight line because this example uses a constant flight-path angle; it is not a vertical climb.

![Animation of three-turn climbing helix](../visuals/climb_helix.gif)

[Open a still image](../visuals/climb_helix.png) · Code: [`example_climb_limit.py`](../examples/example_climb_limit.py)

## 4. A three-leg delivery-area route

The waypoints are fixed in advance: Depot → Customer A → Customer B → Return corridor. Each leg connects a requested position **and heading**. The total distance printed by the script is the sum of the three flyable leg lengths, not the straight-line distance between stops. These are fly-through positions, not modeled landings or payload releases.

![Animation of a three-leg delivery-area route](../visuals/delivery_mission.gif)

[Open a still image](../visuals/delivery_mission.png) · Code: [`example_delivery_mission.py`](../examples/example_delivery_mission.py)

## Run and regenerate

From the `3dDubins` directory:

```bash
python -m pip install -r requirements.txt
python -m examples.example --plot
python -m examples.example_climb_limit --plot
python -m examples.example_arrival_heading --plot
python -m examples.example_delivery_mission --plot
```

Use `--save-visuals` in place of `--plot` to write or update that script's PNG and GIF in `visuals/`. The normal no-flag examples still run without plotting packages. GIF creation uses Matplotlib's Pillow writer; no FFmpeg is needed.

These are geometric illustrations only. They do not check terrain, obstacles, airspace, wind, takeoff/landing, or payload handling. The meaning and limits of the model are in [`KNOWLEDGE_GUIDE.md`](KNOWLEDGE_GUIDE.md).
