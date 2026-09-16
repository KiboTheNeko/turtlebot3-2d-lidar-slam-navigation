# tb3_office_sim — TurtleBot3 SLAM in a 4×8 m office (ROS 2 Jazzy + Gazebo Harmonic)

Headless SLAM mapping of a 4 m × 8 m furnished office arena with a TurtleBot3
Burger (LDS-02 lidar), using `slam_toolbox` in online (async) mode, plus a
post-run verification that the produced map really matches the arena's ground
truth.

The arena, the robot injection, the ROS↔Gazebo bridge, the SLAM tuning, the
closed-loop route driver and the map-comparison tool are all defined **inside
this package** — no external launch files, no hardcoded absolute paths.

| Piece | File in this package |
|-------|----------------------|
| 4×8 m office world | `worlds/office_arena.sdf` |
| World launch (+ robot spawn, bridge, TF helpers) | `launch/office_world.launch.py` |
| slam_toolbox launch (lifecycle autostart, RViz) | `launch/slam_mapping.launch.py` |
| slam_toolbox async tuning for the LDS-02 | `config/mapper_params_online_async.yaml` |
| Pre-configured RViz view | `config/tb3_slam.rviz` |
| Closed-loop furniture-aware route driver | `tb3_office_sim/drive_route.py` |
| Verify map scale vs the 4×8 m SDF | `tb3_office_sim/compare_map.py` |
| Scan re-frame + odom→base_footprint TF helpers | `tb3_office_sim/scan_republisher.py`, `tb3_office_sim/odom_to_tf.py` |
| Saved & verified map | `maps/office_map.pgm`, `maps/office_map.yaml` |
| **v2:** arena-swappable Nav2 auto-mapping (sim→slam→Nav2→explorer) | `launch/autonomous_mapping.launch.py` |
| **v2:** frontier explorer node | `tb3_office_sim/frontier_explorer.py` |
| **v2:** Nav2 tuning + collision monitor | `config/nav2/nav2_params.yaml` |
| **v2:** 5×5 m maze arena + verified map | `worlds/maze_arena.sdf`, `maps/maze_map.pgm`, `maps/maze_map.yaml` |

---

## 1. Prerequisites

- Ubuntu 24.04 with **ROS 2 Jazzy** (`/opt/ros/jazzy`)
- **Gazebo Harmonic** (gz-sim 8). Verify: `gz sim --version` prints `Gazebo Sim ... 8.x`
- Ros packages (all installable via `apt`):

```bash
sudo apt install \
  ros-jazzy-ros-gz ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server \
  ros-jazzy-robot-state-publisher ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard python3-colcon-common-extensions
```

- The vendor TurtleBot3 packages (URDF/xacro assets and teleop node):

```bash
sudo apt install ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-teleop
```

> Gazebo Classic (`gazebo_ros_pkgs`) is **not** used anywhere — see §6 for why.

### Sourcing

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger     # required by the vendor URDF
```

### Build

```bash
cd ~/tb3_office_ws
colcon build --symlink-install     # symlink: source edits take effect on restart (no rebuild)
source install/setup.bash
```

---

## 2. Quick start

```bash
# Terminal 1 — world (GUI on by default; add headless:=true for server-only)
ros2 launch tb3_office_sim office_world.launch.py

# Terminal 2 — slam + RViz (add rviz:=false for headless runs)
ros2 launch tb3_office_sim slam_mapping.launch.py

# Terminal 3 — auto-drive the full furniture-aware route (~3 min), then:
ros2 run tb3_office_sim drive_route

# Terminal 4 — manual alternative:
ros2 run turtlebot3_teleop teleop_keyboard
```

Save + verify the map (see §8–9 for the exact recipe and why it looks the way
it does):

```bash
ros2 run nav2_map_server map_saver_cli -f maps/office_map \
  --ros-args -p map_subscribe_transient_local:=true
ros2 run tb3_office_sim compare_map \
  --pgm maps/office_map.pgm --yaml maps/office_map.yaml
```

Expected result of the last command (see §10):

```
Occupied pixel extent: 82 px x 160 px
Measured room extent:  4.10 m  x  8.00 m      Expected (SDF inner faces): 4.0 m x 8.0 m
PASS: SLAM map scale matches the 4 x 8 m office arena
```

---

## 3. The office arena — SDF anatomy (`worlds/office_arena.sdf`)

The drivable interior is a **4.0 m (x) × 8.0 m (y)** rectangle. The inner wall
faces sit exactly at `x = ±2.0` and `y = ±4.0` (this is the ground truth
`compare_map` checks against). Walls are 0.20 m thick and 1.20 m tall static
boxes. The room comes with office furniture that the LDS-02 must map:

| Model | Pose (center) | Collision box / cyl | Occupies (x, y) |
|-------|---------------|---------------------|-----------------|
| `table_center` | (0, 0.9, 0.3) | 1.0×0.6×0.6 box | x −0.5..0.5, y 0.6..1.2 |
| `desk_west` | (−1.15, −2.2, 0.375) | 1.6×0.9×0.75 box | x −1.95..−0.35, y −2.65..−1.75 |
| `desk_east` | (1.15, 2.2, 0.375) | 1.6×0.9×0.75 box | x 0.35..1.95, y 1.75..2.65 |
| `cabinet_south` | (0, −3.2, 0.8) | 1.2×0.5×1.6 box | x −0.6..0.6, y −3.45..−2.95 |
| `shelf_west` | (−1.68, 0.6, 0.9) | 0.4×2.6×1.8 box | x −1.88..−1.48, y −0.7..1.9 |
| `chair_west` | (−1.62, −1.45, 0) | seat cyl r=0.08 | x −1.70..−1.54, y −1.53..−1.37 |
| `chair_east` | (1.35, 1.45, 0) | seat cyl r=0.08 | x 1.27..1.43, y 1.37..1.53 |

Notes:

- Every model is `<static>true</static>`: the lidar sees it, physics never moves it.
- The desk / table / cabinet layout leaves **one real north–south corridor** at
  `x ≈ −1.3` (east of `shelf_west`, west of the table). `drive_route` uses it
  (twice) to reach the north half of the room (see §7).
- Chair seats were shrunk from r=0.22 → 0.08 and `chair_west` tucked into the
  shelf so that corridor stays passable for a 0.14 m robot.

The **robot is not in this SDF** — a URDF is injected at launch (§4). Physics:

```xml
<physics name="1ms" type="ignored">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

The floor and the wheel/caster `<gazebo>` friction blocks use `mu = mu2 = 2.0`
in `office_world.launch.py` so the 2-wheeled diff-drive robot grips (see §11).

---

## 4. What the world launch does (`launch/office_world.launch.py`)

1. Loads the vendor **TurtleBot3 Burger URDF**, rewrites its mesh URIs to
   `package://turtlebot3_description/...`, and substitutes a **rolling sphere
   caster** for the original drag-heavy box caster (a fixed-joint caster box
   that scrubs the floor adds large friction torque; a 10 mm sphere at toe
   height barely does).
2. Starts `gz-sim` on `office_arena.sdf` (`-r` for run; `-s` when
   `headless:=true`).
3. Publishes static TF from the URDF with `robot_state_publisher`.
4. Spawns the burger into `/world/office_arena` via **`ros_gz_sim create`**
   (service `/world/office_arena/create`, name `turtlebot3_burger`, at the
   origin). The DiffDrive + `gpu_lidar` `<gazebo>` blocks are appended at
   launch time, so they live with the robot, not in the world SDF.
5. Runs the **`parameter_bridge`** (all four topics, §5).
6. Runs `scan_republisher` + `odom_to_tf` — the ROS side of the scan/TF
   pipeline (§5).

Generated `/odom`, `/scan`, `/clock`, `/cmd_vel` all run on **sim time**
(`use_sim_time: true` on every node).

---

## 5. ROS 2 ↔ Gazebo bridge — what and why

**Why `ros_gz_bridge` and not `gazebo_ros_pkgs`?** Gazebo Classic
(`gazebo_ros_pkgs`, `libgazebo_ros_*`) interoperates with gz-sim 8 via a
compatibility shim at best; the *supported* plugin sets for Gazebo Harmonic
are `ros_gz_bridge`, `ros_gz_sim` (spawning), `ros_gz_image`/`ros_gz_interfaces`.
`ros_gz_bridge`'s `parameter_bridge` relays between ROS topics and gz transport
topics with the direction arrow convention: `[` = gz→ROS, `]` = ROS→gz,
`@` = bidirectional.

**The four bridged topics** (all that this headless SLAM stack needs):

| gz transport topic | ROS topic | Direction | Type |
|--------------------|-----------|-----------|------|
| `/clock` | `/clock` | gz→ROS | `rosgraph_msgs/Clock` |
| `/model/turtlebot3_burger/cmd_vel` | `/cmd_vel` | ROS→gz | `geometry_msgs/Twist` |
| `/model/turtlebot3_burger/odometry` | `/odom` | gz→ROS | `nav_msgs/Odometry` |
| `/scan` | `/scan_raw` | gz→ROS | `sensor_msgs/LaserScan` |

Remaps: `cmd_vel` wins the familiar `/cmd_vel` name, `odometry` becomes
`/odom`, and the lidar lands on `/scan_raw` because `scan_republisher` owns
`/scan` (see "scan re-frame" below).

**Why no `/tf` bridge?** Classic Gazebo emitted odometry→base_link TF from
inside the simulator. gz-sim has no such ROS-side bridge, and its
`PosePublisher` publishes **world-frame** poses — bridging those would give
`base_link` a second world parent under a different frame id, corrupting TF.
Instead `/tf` is built entirely on the ROS side:

- `robot_state_publisher` — static `base_footprint→base_link→wheels→base_scan` TF from the URDF,
- `odom_to_tf` — dynamic `odom→base_footprint` from `/odom`.

The slam frame chain is exactly:

```
map --(slam_toolbox)--> odom --(odom_to_tf)--> base_footprint
     --(rsp)--> base_link --(fixed joint)--> base_scan
```

**Scan re-frame (`scan_republisher`).** `sdformat_urdf` flattens fixed joints
when importing the URDF into gz-sim, so the gpu_lidar ends up parented to
`base_footprint` instead of `base_scan`. A raw `/scan_raw` therefore carries
`frame_id = base_footprint`. `scan_republisher` recomputes the beam poses into
the `base_scan` frame (2D rotation about the lidar centre) and republishes as
`/scan` — keeping slam_toolbox's `laser_frame: base_scan` configuration exact.

**Ground truth (not bridged, not part of SLAM).** For verification you can ask
gz directly:

```bash
gz topic -e -t /world/office_arena/pose/info -n 1 --json-output
```

---

## 6. SLAM launch (`launch/slam_mapping.launch.py`)

- Runs `async_slam_toolbox_node` as a **lifecycle node** with
  `namespace=''` and emits the CONFIGURE → ACTIVATE transitions via launch
  lifecycle events (Jazzy's slam_toolbox is lifecycle-managed; without the
  explicit events it stays `unconfigured` forever and never spins).
- Tuning in `config/mapper_params_online_async.yaml` is for the **LDS-02**:
  `resolution: 0.05`, `max_laser_range: 3.5` (the LDS-02's actual ceiling),
  `min_laser_range: 0.12`, `map_update_interval: 5.0`,
  `do_loop_closing: true`, Ceres solver with `SCHUR_JACOBI`.
- `rviz:=false` skips the pre-configured RViz2 (`config/tb3_slam.rviz` shows
  map, laser scan, TF, and odometry).

---

## 7. Driving — closed-loop route (`drive_route.py`)

gz-sim's `DiffDrive` is open-loop: in-place spins lose heading through wheel
slip and straight legs drift. `drive_route` therefore runs each leg as a
two-phase, odometry-closed-loop move:

1. align — rotate in place until `|yaw_target − odom_yaw| < YAW_TOL`,
2. drive — straight at `VX` with small `wz = KP_YAW * yaw_error` correction,
   stopping after integrating wheel-odometry distance.

Tuning: `VX=0.20 m/s`, `WZ_MAX=1.0 rad/s`, `KP_YAW=1.5 1/s`, `YAW_TOL=0.025 rad`,
`DIST_TOL=0.05 m`. The 26-leg route is **furniture-aware**: it walks the south
strip (SW→SE corners), the east strip to y=−1, then drives the only real N–S
corridor (`x≈−1.3`) northward to the NW/NE corners, crosses the north wall, and
returns along the corridor — never entering a desk/table/cabinet/shelf box, and
getting every wall inside the 3.5 m lidar range.

---

## 8. Running the headless mapping pipeline

```bash
# 1) clean slate
pkill -f drive_route; pkill -f async_slam_toolbox; pkill -f parameter_bridge

# 2) world (server only)
ros2 launch tb3_office_sim office_world.launch.py headless:=true

# 3) slam (no RViz)
ros2 launch tb3_office_sim slam_mapping.launch.py rviz:=false
# sanity: ros2 lifecycle get /slam_toolbox  ->  active [3]
#         ros2 topic echo --once /scan --field header.frame_id  ->  base_scan

# 4) auto-drive (~3 min logs progress per leg)
ros2 run tb3_office_sim drive_route
# wait for "route complete - robot stopped"

# 5) save the map (recipe below)
# 6) verify (below)
```

**Map-saving recipe (an important slam_toolbox quirk).** In slam_toolbox
2.8.x the `/map` topic is published with `transient_local` durability and
`updateMap()` **skips publishing while the topic has zero subscribers**
(`get_subscription_count() == 0` returns early). Because `map_update_interval`
is 5 s and `map_saver_cli` gives up after ~2 s, a bare save fails with
`Failed to spin map subscription`. The fix is to keep a subscriber alive long
enough for one publish tick, so the last map gets latched; then the saver
receives it instantly:

```bash
# keep slam mapping: hold a transient_local subscriber
nohup ros2 topic echo /map --qos-durability transient_local > /dev/null 2>&1 &
sleep 7                      # >= one map_update_interval tick

# now the latched map is there for the saver
cd /home/kibo/tb3_office_ws/src/tb3_office_sim
ros2 run nav2_map_server map_saver_cli -f maps/office_map \
  --ros-args -p map_subscribe_transient_local:=true
# expect: "Received a 83 X 161 map @ 0.05 m/pix ... Map saved"
```

`use_map_saver: true` (our config) registers slam_toolbox's own
`/slam_toolbox/save_map` service, but it shells out to the same
`map_saver_cli` — use the recipe above for deterministic saves.

---

## 9. Verifying the map against ground truth (`compare_map.py`)

```bash
ros2 run tb3_office_sim compare_map \
  --pgm maps/office_map.pgm --yaml maps/office_map.yaml
```

What it does: parses the YAML (`resolution`, `origin`), decodes the PGM with a
fixed threshold, computes the bounding box of occupied cells, and compares its
extent to the SDF's inner wall faces (`4.0 × 8.0`, tolerance ±0.2 m). It is
passed to the CLI as plain paths, so no ROS nodes are involved.

Result from the reference run:

```
Map metadata: resolution=0.05 m/cell, origin=(-2.062, -4.054)
PGM: 83 x 161 px, threshold=128
Occupied pixel extent: 82 px x 160 px
Measured room extent:  4.10 m  x  8.00 m
Expected (SDF inner faces): 4.0 m x 8.0 m
Delta: 0.10 m (x)  0.00 m (y)   tolerance +/-0.2 m
PASS: SLAM map scale matches the 4 x 8 m office arena
```

The origin sits right at the room's SW outer corner (walls are 0.2 m thick),
and the occupied extent (4.10 × 8.00 m) matches the 4×8 m drivable interior.

---

## 10. Validated results (this repo's reference run)

- **Physics (fresh sim, ground truth = gz `/world/office_arena/pose/info`):**

  | Maneuver | odom | gz ground truth | ratio |
  |----------|------|-----------------|-------|
  | linear 0.2 m/s × 3 s | 0.550 m | 0.550 m | **1.00** |
  | spin 0.8 rad/s × 3 s | +139.6° | −220.4° (≡ +139.6°) | **1.00** |
  | arc 0.2 + 0.6 rad/s × 3 s | 0.523 m | 0.508 m | **0.97** |

- **Full route end state:** odom `(0.003, −2.001)`, gz `(0.149, −2.065)` —
  within **0.16 m / 3.2°** after 26 legs ≈ 2.5 min of closed-loop driving.
  Residual drift is expected; slam_toolbox corrects at scan level.
- **Map:** `83 × 161 px @ 0.05 m/pix`, origin `(−2.062, −4.054)`, occupied
  extent **4.10 × 8.00 m → PASS** (see §9).

---

## 11. Autonomous mapping v2 — Nav2 + frontier explorer

**v1 (this file, above: `office_world.launch.py` + `drive_route.py`)** drives a
hand-planned 26-leg route and only works in the office. **v2** adds a second,
**arena-generic** pipeline (v1 is untouched and still works) that replaces the
route table with Nav2 navigation + an in-package frontier explorer. Swap the
arena by changing one launch argument — nothing else.

| Piece | File in this package |
|-------|----------------------|
| Arena-swappable full stack (sim→slam→Nav2→explorer) | `launch/autonomous_mapping.launch.py` |
| Frontier explorer node | `tb3_office_sim/frontier_explorer.py` |
| Nav2 tuning + costmaps + collision monitor | `config/nav2/nav2_params.yaml` |
| 5×5 m maze arena (swap target) | `worlds/maze_arena.sdf` |
| RViz2 view (map, costmaps, plans) | `config/tb3_nav2.rviz` |
| Saved & verified maze map | `maps/maze_map.pgm`, `maps/maze_map.yaml` |

### What one launch does (`autonomous_mapping.launch.py`)

```
gz-sim <arena .sdf>  ->  slam_toolbox (/map)  ->  Nav2  ->  frontier_explorer
 (robot spawn)                                        (collision monitor)
```

- **Arena swap:** `world_file:=office_arena.sdf` (4×8 m) or
  `world_file:=maze_arena.sdf` (5×5 m). The world name is read from the SDF
  itself (`_sdf_world_name`), the spawn pose is a launch argument, and the
  explorer has **no route table** — it reads the live `/map`.
- **Nav2 in slam mode:** `bringup_launch.py` with `slam:=True`
  (`use_localization:=False`) — no AMCL, Nav2's costmaps subscribe directly to
  the live `/map` from slam_toolbox. Planner `NavfnPlanner`, controller DWB,
  default BT (spin/back-up recoveries).
- **Collision monitor** stops the robot when anything enters a 0.25 m
  `HardStopZone` circle ahead of it (`action_type: stop`), plus a
  `FootprintApproach` slow-down zone.

```bash
# office arena (v2 default args)
ros2 launch tb3_office_sim autonomous_mapping.launch.py
# maze arena — same stack, zero code changes
ros2 launch tb3_office_sim autonomous_mapping.launch.py \
  world_file:=maze_arena.sdf arena_size:="5 5"
# GUI or manual-goal driving
ros2 launch tb3_office_sim autonomous_mapping.launch.py \
  headless:=false rviz:=true explore:=false
```

### `frontier_explorer.py` — how it explores

Greedy frontier exploration, entirely from the live occupancy grid:

1. Subscribe to `/map` (transient_local) from slam_toolbox.
2. **SAFE free cells** = `FREE (0)` cells at least `SAFE_CLEAR_M=0.30` m from any
   occupied cell (erodes the lethal wall-inflation strip that makes Nav2 abort
   goals on/near walls).
3. **Frontier cells** = `UNKNOWN (−1)` cells with an 8-neighbour SAFE free cell;
   cluster into connected components, and use each component's SAFE free
   centroid as a guaranteed-drivable goal point.
4. Send `NavigateToPose` to the **farthest centroid-preferred** goal.
   *Nearest-first deadlocks in a furniture-cramped arena:* at spawn the whole
   frontier is one ring around the robot whose centroid sits on the robot, so
   Nav2 "reaches" the goal without moving and the map never grows.
5. Aborted/timed-out goals (60 s) mark a 0.35 m bad region; other goals are
   skipped within it. A `FORGIVE_LIMIT=6` budget un-bans regions after retries,
   so a geometrically sealed pocket cannot block completion forever (logged as
   `EXPLORATION COMPLETE (sealed pocket)`).
6. Stop after 3 consecutive map updates with no frontiers →
   `EXPLORATION COMPLETE - no frontiers remaining`, then node idles.

The `/scan → /map` pipeline stays live the whole time: slam keeps building the
map while Nav2 drives, because the costmaps are *subscribers* to the same
`/map` (no map-exchange handoff to time).

### Validated v2 results (reference runs)

**Maze (5×5 m), `world_file:=maze_arena.sdf arena_size:="5 5"`:**

| Phase | Result |
|-------|--------|
| Exploration | 8 goals sent, 78.4 % of arena floor mapped (19.59 m²/25 m²) |
| End condition | `EXPLORATION COMPLETE - no frontiers remaining` |
| Map | `maze_map.pgm/yaml`, 101×101 px @ 0.05 m, origin (−2.567, −2.521) |
| Ground truth check | **PASS** — measured extent 4.95×5.00 m vs 5.0×5.0 m (Δ 0.05 m ≤ ±0.2 m) |
| Goal-nav demo | `NavigateToPose` to (1.8, 1.8) across the maze → **SUCCEEDED** |
| Collision monitor | **zero stop events** during the entire run (startup config only) |

**Office (4×8 m), `world_file:=office_arena.sdf` (default args):**

| Phase | Result |
|-------|--------|
| Exploration | 9 goals sent, 60.7 % of arena floor mapped (19.42 m²/32 m²) |
| End condition | `EXPLORATION COMPLETE - no frontiers remaining` |
| Map | `office_map.pgm/yaml` (overwrites the old v1 map), 81×160 px @ 0.05 m, origin (−2.039, −4.01) |
| Ground truth check | **PASS** — measured extent 4.00×7.95 m vs 4.0×8.0 m (Δ 0.00/0.05 m ≤ ±0.2 m) |
| Collision monitor | **zero stop events** during exploration |

> The maze goal-nav demo doubles as the zero-collision demo (above). A *bonus*
> cross-room office `NavigateToPose` to the SE corner (1.4, −2.4) was attempted
> after exploration — the robot wedged against the north edge of `shelf_west`
> (the same static-furniture contact artifact v1 documented in §12) and the run
> hit its wall-clock budget mid-recovery, so the office demo itself was not
> completed. The maze demo is the required zero-collision evidence.

### Key tuning notes for v2

- **Robot footprint vs corridors.** The TB3 Burger is ~0.105 m radius; the maze
  interior corridors are only **0.85 m** wall-face-to-wall-face. An earlier
  `robot_radius: 0.22` inflated the lethal band to 0.44 m, leaving 0.41 m of
  drivable width — less than the robot — and sealed the corridors. Fixed in
  `config/nav2/nav2_params.yaml`: `robot_radius: 0.15` and
  `inflation_radius: 0.35` in **both** the global and local costmaps (still
  conservative for the burger, and the maze's 0.85 m corridors are navigable).
- **`map_saver_cli`** needs the stack up and a transient_local subscriber
  latched first (see §8's recipe — same quirk applies).

```bash
# save + verify (run while the stack is up)
ros2 run nav2_map_server map_saver_cli -f <maps>_map \
  --ros-args -p map_subscribe_transient_local:=true
ros2 run tb3_office_sim compare_map --pgm maps/<maps>_map.pgm \
  --yaml maps/<maps>_map.yaml --size <W> <H>
```

---

## 12. Notes and known pitfalls

- **LDS-02 range.** `max_laser_range: 3.5` is the physical LDS-02 ceiling. The
  room diagonal (≈8.9 m) and even the long walls (8 m) exceed it, so far walls
  are invisible until the robot approaches them — that is why the route walks
  every wall, and why `do_loop_closing: true` matters when returning.
- **"Healthy sim" only.** gz-sim (Harmonic) **hung twice** in this project
  after a robot got wedged against furniture with the diff-drive fighting a
  contact for minutes (frozen `/clock`/`/scan`/`/odom`, zombie `gz sim`
  process; the ~1.8× odom/gz divergence traced to a *dying sim*, not physics).
  The furniture-aware route + rolling caster were introduced precisely to avoid
  wedging. If `/clock` ever freezes mid-run, restart the whole pipeline; don't
  trust behavioral numbers from a dying sim.
- **Friction package.** Floor plane + wheel `<gazebo reference>` blocks use
  `mu/2 = 2.0` (confirmed honored in the sdformat URDF→SDF conversion). A
  fixed box caster scrubs the floor; it was replaced by a rolling 10 mm sphere.
- **Lifecycle autostart.** slam_toolbox needs `namespace=''` plus launch-event
  CONFIGURE/ACTIVATE — see `slam_mapping.launch.py`.
- **`map_saver_cli`.** Lifecycle-managed in Jazzy and short-timeout by default;
  always use the §8 recipe (persistent transient_local subscriber first).
- **`--symlink-install`** means editing any `.py`/`.yaml`/`.sdf` takes effect on
  the next *restart* — no rebuild needed.

## 13. Cleanup

```bash
pkill -f drive_route; pkill -f async_slam_toolbox; pkill -f parameter_bridge
pkill -f robot_state_publisher; pkill -f rviz2
ps aux | grep 'gz sim' | grep -v grep | awk '{print $2}' | xargs -r kill -9
```