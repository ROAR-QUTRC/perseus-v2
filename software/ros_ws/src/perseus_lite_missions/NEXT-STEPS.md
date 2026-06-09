# Mission Zero — Next Steps

Script (source of truth for the segment flow) is at
`software/assets/missions/rover-robotics-mission-zero-cold-open-script.md`.

## Current state

- Branch: `feature/mission-zero-cold-open` (no commits yet — everything is
  untracked).
- Package `perseus_lite_missions` builds clean. Launch graph resolves.
- Selene Base world (`selene_base.sdf`) in
  `perseus_lite_simulation/worlds/`. Box-primitive habitat with a 0.8 m
  corridor, collapsed panels, debris, leak marker at (9.8, 0). Moon_DEM
  terrain. Launch defaults to this world with spawn at (0, 0, 0.3).
- Broken + reference configs for SLAM and Nav2 are in place. Topics point
  at `/scan`.
- Orchestrator relaunches slam_toolbox with reference params when
  Challenge 1 passes (kill-and-relaunch via subprocess). Persists badges
  to `~/.config/rover-robotics/badges.json`. Publishes mission state +
  badge events on `/mission/state` and `/mission/badge`.
- Checker nodes are functional: map_quality watches the slam YAML on disk,
  goal_reached measures TF distance to the leak pose.

## Resume here

```bash
cd /home/dingo/perseus-lite/software/ros_ws
source install/setup.bash
ros2 launch perseus_lite_missions mission_zero.launch.py --show-args

# To actually launch (Gazebo opens — needs GPU):
ros2 launch perseus_lite_missions mission_zero.launch.py
```

## Remaining work (priority order)

### 1. Web UI page

Create `software/web_ui/src/routes/mission-zero/+page.svelte`. Needs:

- Video player that walks segments from `mission.yaml`.
- Monaco/CodeMirror editor for the two YAML files, highlighting the
  relevant lines per challenge.
- rosbridge subscription to `/mission/state`,
  `/mission/challenge_1/passed`, `/mission/challenge_2/passed`,
  `/mission/badge`.
- Badge overlay on challenge pass.
- Save button that writes the edited YAML back to the orchestrator via
  rosbridge (or via a small REST shim if rosbridge file write is
  awkward — needs a 5-min spike to confirm).

`rosbridge_server` is already launched by `perseus_sim.launch.py`
so the websocket wiring is free.

### 2. `apps.mission-zero` Nix entry

In `flake.nix`, mirror the existing `apps.perseus-lite` pattern. Wrap
should:

1. Source the ROS workspace.
2. Start `ros2 launch perseus_lite_missions mission_zero.launch.py` in
   the background.
3. Serve the web_ui build (`python3 -m http.server` against
   `software/web_ui/build`, or `vite preview`).
4. `xdg-open http://localhost:5173/mission-zero`.

Goal: `nix run github:DingoOz/perseus-lite#mission-zero` opens the
experience in the browser, end-to-end.

### 3. Docs landing page

`docs/source/tutorials/mission-zero.md`. Above-the-fold: the single
`nix run` command, prerequisites (Nix installed, Linux, GPU). Below:
lesson notes from the script (commands + the two YAML diffs) for
readers who prefer text. Link from the docs landing page (commit
`036159fe`).

### 4. Cachix prewarm coverage

Confirm the CI workflow builds `perseus_lite_missions` on push to main
and pushes to `perseus-lite.cachix.org`. If it already builds the
whole ROS workspace, this is free.

### 5. End-to-end run

Walk the full script flow: cold launch, drive, see smeared map, edit
slam params, see clean map, send goal, see corridor blocked, edit nav2
params, watch rover reach the leak. Time it — target under 12 minutes
on a warm cache.

### 6. World geometry polish

The current Selene Base world uses box primitives. For the video
production, you may want:

- Better textures on the habitat walls (Gazebo material scripts).
- A partial roof over parts of the habitat.
- Lighting inside the corridor (point lights at the entrance and leak).
- Dust/particle effects near the collapsed section.
- Contact sensor on the chassis for a no-collisions gate on Challenge 2.

### 7. Nav2 relaunch on Challenge 2

The orchestrator currently only relaunches slam_toolbox (Challenge 1).
Challenge 2 needs the same kill-and-relaunch for the Nav2 stack. This
requires killing the lifecycle_manager + all Nav2 nodes, then
relaunching `perseus_nav_bringup.launch.py` with the reference
nav2_params. Same pattern as the slam relaunch but more nodes.

## Out of scope

- Video production (filming, voiceover, captions).
- Mission 1 onwards.
- Mac/Windows support (call out Linux-only in docs).
- Signed-in skill-tree backend (local badges are enough for Tier 0).

## When ready to commit

```bash
git add software/ros_ws/src/perseus_lite_missions \
        software/ros_ws/src/perseus_lite_simulation/worlds/selene_base.sdf \
        software/assets/missions
git commit -m "feat(missions): Add Mission Zero Selene Base Rescue package

New ament_python package with launch, orchestrator, checkers, broken/
reference configs for SLAM and Nav2 challenges. Selene Base world with
0.8m corridor, habitat geometry, and leak marker."
```

`.claude/` is local config and should stay untracked.
