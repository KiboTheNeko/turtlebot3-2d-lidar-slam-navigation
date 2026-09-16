# GUIDE — read, launch & validate the simulation

Hands-on walkthrough for the TurtleBot3 2D-LiDAR autonomous navigation stack
(ROS 2 Jazzy + Gazebo Harmonic). Intended to be followed **in order** on the
local workspace (`~/tb3_office_ws`). Each section ends with a *Check:* line —
the expected observable result that proves that stage worked.

---

## 0. Environment checklist

Run this first. Everything should print a version / a path, not an error.

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 --version                 # -> ros2 ... jazzy
gz sim --version               # -> Gazebo Sim ... 8.x.x
printenv GZ_SIM_RESOURCE_PATH  # should not be empty
```

**Check:** `gz sim --version` prints **8.x** and no "command not found".

> `gazebo_ros_pkgs` / Gazebo Classic are **not** used anywhere in this stack.

---

## 1. Build

```bash
cd ~/tb3_office_ws
colcon build --symlink-install
source install/setup.bash
```

**Check:** `colcon build` finishes with `Summary: N packages finished`.

> `--symlink-install` means source edits take effect on the next *restart* —
> no rebuild needed.

---

## 2. v2 — autonomous mapping (Nav2 + frontier explorer)

Two arenas are shipped. Run either one; same stack, zero code changes.

### 2a. Office arena (4×8 m) — headless

```bash
# Terminal 1
cd ~/tb3_office_ws && source install/setup.bash
ros2 launch tb3_office_sim autonomous_mapping.launch.py \
  world_file:=office_arena.sdf arena_size:="4 8" headless:=true
```

### 2b. Maze arena (5×5 m) — headless (swap test)

```bash
ros2 launch tb3_office_sim autonomous_mapping.launch.py \
  world_file:=maze_arena.sdf arena_size:="5 5" headless:=true
```

### 2c. With GUI + RViz (visual mode)

```bash
ros2 launch tb3_office_sim autonomous_mapping.launch.py \
  world_file:=maze_arena.sdf arena_size:="5 5" headless:=false rviz:=true
```

### Watching it work

```bash
# explorer decision log (frontier goals / reach / coverage)
ros2 topic echo /rosout --field msg | grep -E "frontier|coverage|COMPLETE"

# or in the launch terminal, look for lines like:
#   -> target frontier (x, y)
#   goal reached | [coverage] free=NN.NN m^2 | goals sent: N
#   EXPLORATION COMPLETE - no frontiers remaining
```

**Check (headless runs):** the terminal shows goal progression
(`target frontier … → goal reached … [coverage] …`) and finally
**`EXPLORATION COMPLETE - no frontiers remaining`**.

---

## 3. Save the map

`map_saver_cli` needs a transient_local subscriber latched first (slam_toolbox
only publishes `/map` while it has subscribers). Keep a subscriber alive for one
`map_update_interval` tick, then save — **while the stack is still up**:

```bash
# Terminal 2 (stack still running)
cd ~/tb3_office_ws/src/tb3_office_sim
nohup ros2 topic echo /map --qos-durability transient_local > /dev/null 2>&1 &
sleep 7                                       # >= one map update tick
ros2 run nav2_map_server map_saver_cli -f maps/maze_map \
  --ros-args -p map_subscribe_transient_local:=true
```

**Check:** prints `Map saved successfully` and `maps/maze_map.pgm`
+ `maps/maze_map.yaml` exist.

---

## 4. Validate the map (ground truth)

The `compare_map` tool decodes the `.pgm/.yaml` (no ROS nodes needed) and
compares the occupied extent against the arena's real inner-wall dimensions.

```bash
# maze — expected 5 x 5 m
ros2 run tb3_office_sim compare_map \
  --pgm maps/maze_map.pgm --yaml maps/maze_map.yaml --size 5 5; echo "exit=$?"

# office — expected 4 x 8 m
ros2 run tb3_office_sim compare_map \
  --pgm maps/office_map.pgm --yaml maps/office_map.yaml --size 4 8; echo "exit=$?"
```

**Check:** output ends with **`PASS: SLAM map scale matches the … arena`** and
**`exit=0`**. A `FAIL`/wonky extent means the coverage was incomplete.

Reference (verified runs, 2026-09-16):

| Arena | measured | expected | delta | result |
|-------|----------|----------|-------|--------|
| maze 5×5 | 4.95 × 5.00 m | 5.0 × 5.0 m | ≤ 0.05 m | PASS |
| office 4×8 | 4.00 × 7.95 m | 4.0 × 8.0 m | ≤ 0.05 m | PASS |

---

## 5. Optional — manual goal navigation (maze solving demo)

With the mapped stack still up (explorer now idle in `DONE`), drive a
cross-room goal manually:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.8, y: 1.8, z: 0.0}, orientation: {w: 1.0}}}}"
```

**Check:** prints **`Goal finished with status: SUCCEEDED`**. The robot must
thread the maze corridors (multiple turns) to reach the NE pocket.

---

## 6. Collision-monitor check (zero-stop evidence)

The HardStopZone (0.25 m) should **never** hard-stop during a healthy run —
it only fires on imminent contact.

```bash
# count stop events in the launch log (expect 0 besides startup config lines)
grep -c "HardStopZone.*Creating" <launch_log>   # 2 at startup only
grep -ci "HardStopZone.*stop" <launch_log>      # 0
```

**Check:** the second grep prints **0** — the robot approached every wall and
furniture using costmaps + slow-down alone.

---

## 7. v1 — classic flow (office only, unchanged legacy)

```bash
# Terminal 1 — world + spawn + bridge
ros2 launch tb3_office_sim office_world.launch.py

# Terminal 2 — SLAM + RViz
ros2 launch tb3_office_sim slam_mapping.launch.py

# Terminal 3 — closed-loop 26-leg route driver
ros2 run tb3_office_sim drive_route
# wait for: "route complete - robot stopped"
```

**Check:** the route driver logs each of the 26 legs and finishes with
`route complete`.

---

## 8. Sanity checks (when something looks wrong)

| Symptom | Check |
|---------|-------|
| SLAM never starts mapping | `ros2 lifecycle get /slam_toolbox` → `active [3]`; `ros2 topic echo --once /scan --field header.frame_id` → `base_scan` |
| Explorer stuck at "waiting for Nav2" | `ros2 action list` contains `/navigate_to_pose`; give the `TimerAction(10 s)` its delay |
| Map save fails "Failed to spin map subscription" | redo §3 — the transient_local latch was missing |
| Robot wedged / frozen odom against furniture | Gazebo contact-pin sim artifact → restart the whole pipeline (clean with §9); odom↔gz check: `ros2 topic echo /odom` vs `gz topic -e -t /world/<world>/pose/info -n 1` |
| Corridors not navigable after footprint param edits | restart (symlink install) — params are read at node start |

---

## 9. Cleanup

```bash
pkill -f drive_route; pkill -f async_slam_toolbox; pkill -f parameter_bridge
pkill -f robot_state_publisher; pkill -f rviz2
ps aux | grep 'gz sim' | grep -v grep | awk '{print $2}' | xargs -r kill -9
# verify: ps aux | grep -c 'gz sim' -> 0
```

---

## Reference numbers (short form)

- Maze 5×5: 8 goals, 19.59 m² (78.4 %), EXPLORATION COMPLETE, map **PASS**,
  goal-nav demo **SUCCEEDED** (~29 s), **0** collision stops.
- Office 4×8: 9 goals, 19.42 m² (60.7 %), EXPLORATION COMPLETE, map **PASS**,
  **0** collision stops.

Full evidence (log lines, checklists): see `VERIFICATION_v2.md`.
Architecture & tuning: see `README.md` (§11 for v2).