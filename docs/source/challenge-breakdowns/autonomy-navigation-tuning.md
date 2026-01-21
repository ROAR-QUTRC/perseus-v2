# Navigation Constraints and Tuning

## Costmap Parameters

| Parameter | Local | Global | Depends On |
|-----------|-------|--------|------------|
| Resolution | 0.05 m | 0.05 m | Map resolution |
| Robot radius | 0.12 m | 0.12 m | **Chassis dimensions** |
| Inflation radius | 0.55 m | 0.35 m | Robot radius |
| Update frequency | 5.0 Hz | 1.0 Hz | Sensor rate |
| Rolling window | Yes | No | — |
| Size | 3m × 3m | Map size | — |

### Costmap Layers

**Local Costmap:**
- `obstacle_layer` — Dynamic obstacles from `/scan`
- `inflation_layer` — Safety buffer around obstacles

**Global Costmap:**
- `static_layer` — Pre-built map
- `obstacle_layer` — Dynamic obstacles
- `inflation_layer` — Safety buffer

## Controller Parameters

### DWB Local Planner

| Parameter | Value | Depends On |
|-----------|-------|------------|
| `max_vel_x` | 0.2 m/s | Kinematic limits |
| `max_vel_theta` | 0.8 rad/s | Kinematic limits |
| `acc_lim_x` | 2.0 m/s² | Motor capability |
| `acc_lim_theta` | 2.5 rad/s² | Motor capability |
| `sim_time` | 1.5 s | Velocity limits |
| `vx_samples` | 20 | Trajectory resolution |
| `vtheta_samples` | 40 | Trajectory resolution |

### Goal Tolerances

| Parameter | Value | Notes |
|-----------|-------|-------|
| `xy_goal_tolerance` | 0.1 m | Position accuracy |
| `yaw_goal_tolerance` | 0.2 rad | Heading accuracy (~11°) |

### Trajectory Critics

| Critic | Scale | Purpose |
|--------|-------|---------|
| `RotateToGoal` | 32.0 | Align with goal heading |
| `Oscillation` | — | Prevent back-and-forth |
| `BaseObstacle` | 0.05 | Obstacle avoidance |
| `PathAlign` | 32.0 | Stay on global path |
| `PathDist` | 32.0 | Minimise path deviation |
| `GoalAlign` | 24.0 | Align approach to goal |
| `GoalDist` | 24.0 | Progress toward goal |

## AMCL Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Max particles | 2000 | Increased for larger environments |
| Min particles | 500 | Prevents particle depletion |
| Laser model | `likelihood_field` | Standard for 2D LiDAR |
| Max beams | 60 | Computational efficiency |
| Laser max range | 12.0 m | M2M2 effective range |
| Update min distance | 0.5 m | Scaled for robot size |
| Update min angle | 0.3 rad | Scaled for robot size |
| Recovery alpha fast | 0.1 | Enables recovery behaviour |
| Recovery alpha slow | 0.001 | Gradual recovery |

## SLAM Toolbox Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Map resolution | 0.02 m | High detail |
| Max laser range | 8.0 m | Indoor mapping |
| Min travel distance | 0.1 m | New node trigger |
| Min travel heading | 0.2 rad | New node trigger |
| Loop closure | Enabled | Corrects drift |
| Loop search distance | 2.0 m | Search radius |

## Recovery Behaviours

Configured in `perseus_nav_params.yaml`:

| Behaviour | Plugin | Purpose |
|-----------|--------|---------|
| `spin` | `nav2_behaviors::Spin` | Rotate to clear obstacles |
| `backup` | `nav2_behaviors::BackUp` | Reverse from obstacle |
| `drive_on_heading` | `nav2_behaviors::DriveOnHeading` | Move forward fixed distance |
| `wait` | `nav2_behaviors::Wait` | Pause before retry |
| `assisted_teleop` | `nav2_behaviors::AssistedTeleop` | Manual intervention |

### Recovery Parameters

| Parameter | Value |
|-----------|-------|
| Max rotational velocity | 0.8 rad/s |
| Min rotational velocity | 0.2 rad/s |
| Rotational acceleration | 2.5 rad/s² |
| Simulate ahead time | 2.0 s |

## Collision Monitor

The collision monitor provides an additional safety layer:

| Zone | Type | Action | Radius/Size |
|------|------|--------|-------------|
| `PolygonStop` | Circle | Stop | 0.15 m |
| `PolygonSlow` | Polygon | Slowdown 50% | 0.15 m box |
| `FootprintApproach` | Polygon | Approach | Robot footprint |

## Tuning Guidelines

### If robot moves too slowly
- Increase `max_vel_x` (up to hardware limit)
- Decrease `sim_time` for faster response

### If robot oscillates
- Decrease velocity limits
- Increase `Oscillation` critic weight
- Reduce acceleration limits

### If robot hits obstacles
- Increase `BaseObstacle` critic scale
- Increase inflation radius
- Reduce `max_vel_x`

### If localisation jumps
- Increase AMCL `min_particles`
- Check TF tree completeness
- Verify sensor frame alignment
