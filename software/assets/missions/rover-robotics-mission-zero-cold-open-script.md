# Rover Robotics — Video Script

**Mission Zero — The Cold Open ("Selene Base Rescue")** Tier 0 (free flagship hook) · Phase 1 (Seed) · Mode: Sim · Target length: \~10.5 min · Repo tag: `m00-cold-open`

The hook. The viewer is dropped straight into the simulator with a problem to solve, no installation and no lecture, and walks away in ten minutes having driven a robot, repaired a real map, and solved a real navigation problem — using the exact parameters professionals tune. The theory is deliberately deferred; this video exists to manufacture the questions the rest of the campaign answers.

---

## Production notes

- **Shape:** cinematic cold open, then screencast-driven challenges, framed by the rescue story. Each challenge segment shows the broken behaviour, the fix, then the win.  
- **Brand:** navy/orange, no icons, Australian spelling throughout. Calm, confident voice — stakes are high but the host is reassuring.  
- **Intro/outro:** hold the standard series sting until after the first win (Segment 5), so we open cold. Close on the standard outro with a clear call to action.  
- **On-screen:** Gazebo (the Selene Base lunar scene), RViz2 (map \+ costmap), a text editor for the two config files, and a terminal. A small skill-tree overlay appears on each badge unlock.  
- **Scenario assets:** this lesson ships the `selene_base` Gazebo scenario, the two deliberately broken config files, a reference solution, and the checker node that confirms each win before a badge unlocks.  
- **Accessibility:** burn-in captions; every command and config change shown also ships as copy-and-paste-able text in the lesson notes.  
- **Distro:** ROS2 is the pinned series distribution (Jazzy on Ubuntu 24.04 LTS in the reference recording image). Keep version strings off-screen so the footage ages well.

---

## Script

### Segment 1 — Cold open: the rescue (0:00–1:00)

**VISUAL:** Black. A single line of telemetry text types on: *SELENE BASE — HABITAT C — PRESSURE: FALLING*. Cut to the Gazebo lunar scene: grey regolith, a collapsed colony building, a small six-wheeled rover sitting in the dust. Slow push-in. Muted alarm tone under the voice.

**NARRATION:**

Selene Base has gone dark. A meteorite came through the roof of Habitat C, and somewhere inside that collapsed building there's a leak — the module is losing pressure by the minute. It is far too dangerous to send a person in.

But the colony's maintenance rover still works. Just barely. Its systems are mis-configured and its sense of itself is scrambled — so right now it can't map, and it can't navigate. Your job, in the next ten minutes, is to bring it back to life and drive it to that leak.

No installation. No theory. The robot is already running in front of you. Let's go.

---

### Segment 2 — First contact: drive it (1:00–2:10)

**VISUAL:** Screencast. The Gazebo window is live. Bring up a teleop terminal beside it.

**NARRATION:**

First, let's prove the rover is alive. We'll take manual control:

**ON-SCREEN:**

ros2 run teleop\_twist\_keyboard teleop\_twist\_keyboard

**NARRATION:**

Click that terminal and use the keys to drive.

**VISUAL:** Press the drive keys. The rover rolls forward across the lunar surface, turns, kicks up a little dust. Hold on this for a beat — the satisfying first move.

**NARRATION:**

That's it — you're driving a real robot. Every key press is becoming a velocity command the rover obeys. It moves, so the hardware's fine. The problem is everything it's supposed to do on its own: it can't see the building as a map, and it won't plan a path. So that's what we fix — two faults, two quick wins, and we're at the leak.

---

### Segment 3 — The smeared map (the break) (2:10–3:00)

**VISUAL:** Open RViz2 with the map view. Start the mapping launch.

**NARRATION:**

To reach the leak, the rover has to build a map of the corridors as it drives. Let's switch its mapper on:

**ON-SCREEN:**

ros2 launch selene\_base mapping.launch.py

**VISUAL:** Drive the rover slowly down Corridor A. In RViz2 the map comes out as a thick, doubled-up smear — walls blur into each other, the corridor appears to close in on itself. It looks broken, because it is.

**NARRATION:**

That is a mess. The walls are doubling up and the corridor is folding in on itself — there's no way to navigate against a map like this. The rover's mapper has been handed two bad settings, and we only need to change those two values to fix it.

---

### Segment 4 — Challenge 1: fix the map (3:00–4:30)

**VISUAL:** Open the mapper config in the editor. Highlight the two lines as they're discussed.

**NARRATION:**

Open the mapper's configuration — it's the SLAM toolbox parameter file that shipped with the mission. Two values are wrong. The map `resolution` is far too coarse, so every wall is drawn as a fat, blurry block. And the laser's `max_laser_range` is far too short, so the rover only ever sees the nearest slice of each wall — and those slices don't line up as it moves, which is what gives us the smear.

**ON-SCREEN (before — `config/slam_params.yaml`):**

slam\_toolbox:

  ros\_\_parameters:

    resolution: 0.30          \# far too coarse: walls blur into thick blocks

    max\_laser\_range: 3.0      \# far too short: only the nearest wall slice is seen

**NARRATION:**

Tighten the resolution to five centimetres so the walls come out thin and crisp, and open the laser range up so the rover sees the whole corridor at once:

**ON-SCREEN (after — `config/slam_params.yaml`):**

slam\_toolbox:

  ros\_\_parameters:

    resolution: 0.05          \# 5 cm cells: crisp, single-width walls

    max\_laser\_range: 12.0     \# see the full corridor in one sweep

**NARRATION:**

Save it, restart the mapper, and drive Corridor A again.

**VISUAL:** Relaunch mapping. Drive down the corridor. The map snaps into a clean, single-walled, straight passage. A small checker toast appears: *Corridor A mapped — clean.* The first skill-tree badge lights up in the overlay.

**NARRATION:**

There it is. Clean walls, a straight corridor, a map you can actually navigate. First badge unlocked. You just tuned a real SLAM mapper — the same two parameters a working robotics engineer reaches for first.

---

### Segment 5 — Brand beat: what just happened (4:30–5:05)

**VISUAL:** Series intro sting lands here (short). Then back to host as a brief talking-head, or stylised text over the lunar scene.

**NARRATION:**

This is Rover Robotics. Notice what we did not do — we didn't sit through a lecture on how SLAM works. You hit a real problem, changed real settings, and saw it fix itself. The "why" comes later, and it'll make a lot more sense now that you've felt it. One fault down. One to go — and this is the one standing between us and the leak.

---

### Segment 6 — The corridor that will not open (the break) (5:05–6:00)

**VISUAL:** Start the navigation launch. Switch RViz2 to show the costmap. Send a goal at the habitat.

**NARRATION:**

Now we ask the rover to drive itself. We'll bring up the navigation stack and send it a goal at the habitat door:

**ON-SCREEN:**

ros2 launch selene\_base navigation.launch.py

**VISUAL:** In RViz2, use the Nav2 goal tool to drop a goal at the leak. Nav2 reports the path blocked. The rover does not move. Show the costmap: the corridor is flooded edge-to-edge with high-cost (inflated) colour — no clear channel.

**NARRATION:**

Nothing. Nav2 says the path is blocked and the rover just sits there. But look at the corridor — it's eight hundred millimetres wide, and the rover is far narrower than that. So why won't it go? Because the rover has been told it's much bigger than it really is. Look at the costmap: that whole corridor is painted as too-dangerous-to-enter. The planner genuinely believes there's no room.

---

### Segment 7 — Challenge 2: open the corridor (6:00–8:15)

**VISUAL:** Open the Nav2 config in the editor. Highlight the two settings.

**NARRATION:**

Open the navigation configuration. Two things are wrong, and they compound. First, the rover's footprint — its `robot_radius` — is set enormous, far bigger than the real machine. Second, the `inflation_radius`, the safety buffer the planner paints around every wall, is so large it floods the entire corridor. Between them, there's no room left to drive.

**ON-SCREEN (before — `config/nav2_params.yaml`):**

global\_costmap:

  global\_costmap:

    ros\_\_parameters:

      robot\_radius: 0.85          \# rover described far larger than it is

      inflation\_layer:

        inflation\_radius: 0.75    \# safety buffer floods the 0.8 m corridor

local\_costmap:

  local\_costmap:

    ros\_\_parameters:

      robot\_radius: 0.85

      inflation\_layer:

        inflation\_radius: 0.75

**NARRATION:**

Set the footprint to the rover's true radius — about twenty-eight centimetres — and bring the inflation buffer down so it protects the walls without sealing the passage. Do it in both the global and local costmaps:

**ON-SCREEN (after — `config/nav2_params.yaml`):**

global\_costmap:

  global\_costmap:

    ros\_\_parameters:

      robot\_radius: 0.28          \# the rover's true radius

      inflation\_layer:

        inflation\_radius: 0.25    \# protects the walls, leaves a drivable channel

local\_costmap:

  local\_costmap:

    ros\_\_parameters:

      robot\_radius: 0.28

      inflation\_layer:

        inflation\_radius: 0.25

**NARRATION:**

Save it, restart navigation, and send the goal again.

**VISUAL:** Relaunch navigation. Re-send the goal. The costmap inflation shrinks back to thin bands along the walls, leaving a clear channel down the middle. A green path appears. The rover sets off, threads the corridor, rounds the corner, and rolls to a stop directly at the leak. Checker toast: *Habitat reached — zero collisions.* The second badge lights up.

**NARRATION:**

And there it goes. The planner finds the channel, the rover threads the corridor, takes the corner, and stops right at the leak. No collisions. Second badge unlocked — you just got an autonomous robot through a gap it swore it couldn't fit.

---

### Segment 8 — Payoff (8:15–9:00)

**VISUAL:** Hold on the rover parked at the leak, the habitat behind it. RViz2 shows the clean map with the rover's path traced through it. The alarm tone fades.

**NARRATION:**

Ten minutes ago this rover couldn't see and couldn't move on its own. Now it's mapped a collapsed building and driven itself to the one spot that matters. You did that by changing four numbers — and every one of them is a real setting that real robots depend on.

Which raises the obvious questions. Why did those two map settings matter so much? How does the rover actually know where it is inside that map? And why is a "safety buffer" the thing that decides whether it can fit through a door? That's the rest of the rescue — and it's where you'll learn to do all of this properly, from scratch.

---

### Segment 9 — Skill tree and call to action (9:00–10:30)

**VISUAL:** The skill-tree overlay fills the frame. Two badges glow (Mapping and Navigation, first tier). Branches fan out to locked nodes ahead — ROS2 foundations, building the rover, sensing and odometry, full SLAM, full Nav2, the hardware build. The tree is clearly non-linear.

**NARRATION:**

Everything you just touched is the first rung on this skill tree. It's not a straight line — it branches, and it points you to what makes sense to learn next based on what you've already unlocked. Start at the beginning and you'll build this rover yourself, in simulation, for free: install the tools, model the machine, and earn your way back to mapping and navigation knowing exactly how they work.

And here's the part that matters: the entire stack you'll learn in simulation is the same software that runs on the real rover. So when you're ready for hardware, you're not starting over — you're deploying a rescue you've already pulled off.

The first missions are free. Begin the Selene Base Rescue at the link below, and let's bring this colony back online.

**VISUAL:** Standard outro card. Link to the free foundations / sign-up. End.

---

## Lesson notes (ship with the video)

Commands used, in order:

\# Source ROS2 (the pinned series distribution)

source /opt/ros/jazzy/setup.bash

\# Drive the rover manually (proves it is alive)

ros2 run teleop\_twist\_keyboard teleop\_twist\_keyboard

\# Challenge 1 — mapping

ros2 launch selene\_base mapping.launch.py

\#   edit config/slam\_params.yaml, then relaunch

\# Challenge 2 — navigation

ros2 launch selene\_base navigation.launch.py

\#   send a goal in RViz2 (Nav2 Goal tool)

\#   edit config/nav2\_params.yaml, then relaunch and re-send the goal

**Challenge 1 — the smeared map.** In `config/slam_params.yaml`, change `resolution` from `0.30` to `0.05` and `max_laser_range` from `3.0` to `12.0`. The map changes from a doubled-up smear to clean, single-width walls.

**Challenge 2 — the blocked corridor.** In `config/nav2_params.yaml` (both `global_costmap` and `local_costmap`), change `robot_radius` from `0.85` to `0.28` and `inflation_radius` from `0.75` to `0.25`. The costmap stops flooding the 0.8 m corridor and the planner finds a route to the leak.

**Why this works.** In ten minutes the viewer drives a robot, repairs a real SLAM map, and solves a real Nav2 navigation problem using the exact parameters professionals tune — without being taught the underlying theory first. The cold open manufactures the questions (why those values matter, how the rover localises, what inflation really does) that the campaign exists to answer, and previews its two pillars: mapping (SLAM) and navigation (Nav2).  
