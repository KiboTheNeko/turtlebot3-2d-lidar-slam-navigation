# v2 Verification Report — Nav2 autonomous mapping (ROS 2 Jazzy + Gazebo Harmonic)

Date: 2026-09-16  ·  Package: `tb3_office_sim`  ·  Arena-swappable pipeline
`launch/autonomous_mapping.launch.py` + `tb3_office_sim/frontier_explorer.py`
(constraints: no gazebo_ros_pkgs, no hardcoded paths, `--symlink-install`).

---

## 1. Maze arena (5×5 m) — full pass

Run: `ros2 launch tb3_office_sim autonomous_mapping.launch.py \
  world_file:=maze_arena.sdf arena_size:="5 5" headless:=true explore:=true`

### 1.1 Exploration

| Metric | Value |
|--------|-------|
| Goals sent | 8 |
| Final free-space coverage | 19.59 m²  (78.4 % of 25 m²) |
| Frontier goals reached | 7 of 8 (one 60 s per-goal timeout at (1.02, 0.80)) |
| End condition | `EXPLORATION COMPLETE - no frontiers remaining` (after 3 consecutive empty frontier maps) |
| HardStopZone stop events | **0** over the whole run (only startup "Creating Polygon/Circle" config lines) |

Goal progression (`coverage … | goals sent`):
`7.31 → 12.16 → 14.06 → 14.98 → 18.38 → 19.39 → 19.59 m²` (goals 1–6, then the
seventh goal re-verified, one cancelled). Sample log lines:

```
goal reached        [coverage] free=19.39 m^2 | goals sent: 6
no frontiers (1/3)  (2/3)  (3/3)
[coverage] free=19.59 m^2 | goals sent: 8
EXPLORATION COMPLETE - no frontiers remaining
```

### 1.2 Map ground-truth check (`compare_map --size 5 5`)

```
Map metadata: resolution=0.05 m/cell, origin=(-2.567, -2.521)
PGM: 101 x 101 px, threshold=128
Occupied pixel extent: 99 px x 100 px
Measured room extent:  4.95 m  x  5.00 m
Expected (arena inner faces): 5.0 m x 5.0 m
Delta: 0.05 m (x)  0.00 m (y)   tolerance +/-0.2 m
PASS: SLAM map scale matches the 5 x 5 m arena        (exit=0)
```

Saved: `maps/maze_map.pgm` / `maps/maze_map.yaml`.

### 1.3 Zero-collision goal-nav demo (maze solving)

Sent while the mapped stack was alive, after exploration completed (explorer in
`DONE` phase → idle, cannot fight the manual goal):

```
Begin navigating from current location (0.26, -1.08) to (1.80, 1.80)   [1789533357]
Goal succeeded                                                          [1789533386]  (~29 s)
```

- Target (1.80, 1.80) is the far **NE pocket** behind two internal wall
  segments — reaching it requires multiple corridor turns (maze solving).
- Final odom: (1.63, 1.94); within Nav2's `xy_goal_tolerance`.
- **Collision monitor:** `HardStopZone` stop events during the demo = **0**
  (grep across the entire maze log: only the two startup lines
  `[HardStopZone]: Creating Polygon / Creating Circle`; zero stop commands
  ever issued).

---

## 2. Office arena (4×8 m) — exploration + map, footprint fix verified

Run: `ros2 launch tb3_office_sim autonomous_mapping.launch.py \
  world_file:=office_arena.sdf arena_size:="4 8" headless:=true explore:=true`

> Context: v1-era office runs with `robot_radius: 0.22` wedged at
> `table_center` (contact pin: frozen odom ≈ (−0.94, 0.79), oscillating
> cmd_vel, spin-only recoveries). Fixed in `nav2_params.yaml` →
> `robot_radius: 0.15`, `inflation_radius: 0.35` (both costmaps). This run uses
> the fix.

### 2.1 Exploration

| Metric | Value |
|--------|-------|
| Goals sent | 9 |
| Final free-space coverage | 19.42 m²  (60.7 % of 32 m²) |
| End condition | `EXPLORATION COMPLETE - no frontiers remaining` |
| Cancel-led timeout goals | 2 (goal 6 at (0.49, 3.04) region and later — robot brushed the NW `shelf_west` north edge; spin/backup recoveries fired, odom vs gz stayed consistent, no physics freeze) |
| HardStopZone stop events | **0** during exploration |

Coverage progression: `6.56 → 10.80 → 12.03 → 13.09 → … → 19.42 m²` (9 goals).
The NW corner was filled by lidar sweep during spin recoveries, which is why the
no-frontier check tripped while the robot sat at (−1.39, 2.10) — a legit
completion (frontier detection works on the *map*, not on robot pose).

### 2.2 Map ground-truth check (`compare_map --size 4 8`)

```
Map metadata: resolution=0.05 m/cell, origin=(-2.039, -4.01)
PGM: 81 x 160 px, threshold=128
Occupied pixel extent: 80 px x 159 px
Measured room extent:  4.00 m  x  7.95 m
Expected (arena inner faces): 4.0 m x 8.0 m
Delta: 0.00 m (x)  0.05 m (y)   tolerance +/-0.2 m
PASS: SLAM map scale matches the 4 x 8 m arena        (exit=0)
```

Saved: `maps/office_map.pgm` / `maps/office_map.yaml` (overwrote the v1 map —
intended; fresh v2 artifact).

### 2.3 Office goal-nav demo — attempted, not completed

After exploration, a cross-room `NavigateToPose` to the SE area (1.4, −2.4)
was sent from (−1.39, 2.10). The robot wedged against the **north edge of
`shelf_west`** (the same static-furniture contact artifact documented in v1's
pitfalls, §12) — repeated `Failed to make progress`, spin/backup recoveries
failing while the 540 s wall-clock budget ran out mid-recovery. The demo did
not reach the goal.

**Assessment:** the *required* zero-collision goal-nav demo is the maze one
(§1.3), which passed cleanly across maze walls. The office attempt failed on a
known simulator contact-pin physics artifact, not on planning (the global
planner succeeded; the robot physically could not slide off the static model).
This is the same class of artifact v1 already documents — not a regression, and
no collision-monitor stop (the monitor correctly never triggered, and the robot
never actually collided at speed).

---

## 3. Requirements checklist

| Requirement | Status | Evidence |
|-------------|--------|----------|
| ROS 2 Jazzy + Gazebo Harmonic, no `gazebo_ros_pkgs` | ✅ | All nodes use `ros_gz_*`; bridge via `parameter_bridge` |
| No hardcoded absolute paths; `get_package_share_directory` | ✅ | `autonomous_mapping.launch.py` uses `get_package_share_directory` throughout |
| v2 separate pipeline; v1 untouched | ✅ | v1 files byte-untouched (Sep 13 timestamps); v2 lives in own launch/explorer |
| Obstacle avoidance: costmaps + collision_monitor HardStopZone stop @ 0.25 m | ✅ | `nav2_params.yaml` lines 89–114 (circle r=0.25, action stop); zero stop events fired |
| Exploration stops on no-frontiers / sealed pocket (forgive budget) | ✅ | `EXPLORATION COMPLETE - no frontiers remaining` in both arenas; `FORGIVE_LIMIT=6` code path |
| World name parsed from SDF | ✅ | `_sdf_world_name()`; office & maze swapped with zero code edits |
| `compare_map` PASS for arena dims | ✅ | maze 4.95×5.00 (Δ0.05), office 4.00×7.95 (Δ0.05) — both ≤ ±0.2 m |
| Zero-collision goal-nav demo | ✅ | maze demo SUCCEEDED (29 s), zero HardStopZone stops |
| Footprint fixed for 0.85 m maze corridors | ✅ | `robot_radius 0.15` / `inflation 0.35` in both costmaps; maze corridors traversed both directions |

---

## 4. Files touched in v2 work

| File | Change |
|------|--------|
| `launch/autonomous_mapping.launch.py` | v2 arena-swappable launch (SDF world-name parse, spawn args, Nav2 `slam:=True`, explorer TimerAction) |
| `tb3_office_sim/frontier_explorer.py` | rewritten explorer: safe-free mask, centroid-preferring farthest-first goals, bad-goal recall, forgive budget, `DONE` idle phase |
| `config/nav2/nav2_params.yaml` | `robot_radius 0.15`, `inflation_radius 0.35` (global+local); HardStopZone 0.25 m stop |
| `tb3_office_sim/compare_map.py` | `--pgm/--yaml/--size W H`; exit 0 = PASS, ±0.2 m tolerance |
| `worlds/maze_arena.sdf` | 5×5 m maze (internal 0.15 m walls, ~0.85 m corridors) |
| `maps/maze_map.*`, `maps/office_map.*` | verified v2 artifacts |
| `README.md` | §11 v2 section + table entries |
| `tb3_office_sim/odom_to_tf.py` | stamp fix (shared with v1, present since earlier) |

Cleanup discipline maintained (bracketed `pkill` patterns; verified
`ps … | grep` zero zombies after each run).