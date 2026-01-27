# Autonomy and Mapping

```{toctree}
:maxdepth: 1
:hidden:

autonomy-quick-start
autonomy-architecture
autonomy-interface-contracts
autonomy-physical-constants
autonomy-navigation-tuning
autonomy-launch-reference
autonomy-preflight-checklist
autonomy-troubleshooting
```

This section provides the definitive reference for Perseus autonomy operations. It serves as both a repeatable bring-up procedure and a contract specification—changes to items marked as **contracts** require coordination with the autonomy team before implementation.

**2026 Team Members**

- Nigel
- Natalia Hoyos Gonzalez
- Jasper
- Lachlan Ikeguchi

**Document Structure**

| Page                                                    | Description                                                  |
| ------------------------------------------------------- | ------------------------------------------------------------ |
| [Quick Start](autonomy-quick-start.md)                  | Copy-paste commands for bring-up                             |
| [Architecture](autonomy-architecture.md)                | System design, TF tree, data flow                            |
| [Interface Contracts](autonomy-interface-contracts.md)  | Topics, frames, and hardware interfaces that must not change |
| [Physical Constants](autonomy-physical-constants.md)    | Geometry and sensor specs                                    |
| [Navigation Tuning](autonomy-navigation-tuning.md)      | Nav2, AMCL, and SLAM parameters                              |
| [Launch Reference](autonomy-launch-reference.md)        | Detailed launch file documentation                           |
| [Pre-Flight Checklist](autonomy-preflight-checklist.md) | Test-day verification steps                                  |
| [Troubleshooting](autonomy-troubleshooting.md)          | Common issues and debug commands                             |

**Who Should Read This**

| Subteam     | Start Here                                                                                                   |
| ----------- | ------------------------------------------------------------------------------------------------------------ |
| Software    | All pages                                                                                                    |
| Electrical  | [Interface Contracts](autonomy-interface-contracts.md), [Physical Constants](autonomy-physical-constants.md) |
| Mechanical  | [Physical Constants](autonomy-physical-constants.md)                                                         |
| All members | [Quick Start](autonomy-quick-start.md), [Architecture](autonomy-architecture.md)                             |

**When to Update**

- **Before** any mechanical, electrical, or software change affecting sensors, drivetrain, or frames
- **After** test days with observed discrepancies
- **After** any calibration or tuning session

## Technical Notes

- Map updates occur only after Perseus has executed sufficient movement or rotation to trigger an update
- Update trigger parameters are configurable in `config/slam_toolbox_params.yaml`
- The system utilises ROS2's SLAM Toolbox for mapping functionality

## Mapping & Autonomous Task - Australian Rover Challenge 2025

- **Goal:** Autonomous exploration and mapping, navigation to specific landmarks by Perseus.

### Points break-down

| Activity                                  | Points                  |
| ----------------------------------------- | ----------------------- |
| - Leave the Start Area Autonomously       | 5 points                |
| - For each placard imaged and relayed     | 6 points per placard    |
| - Location within 300mm of true position  | 5 points per cube       |
| - Location within 600mm of true position  | 2 points per cube       |
| - Autonomous phase bonus                  | Double the above points |
| - Design and justification for navigation | Up to 5 points          |
| - Mapping system design                   | Up to 5 points          |
| - Details and visualisation of the map    | Up to 15 points         |
| **Total Possible Points**                 | 100 points              |

### Autonomous Phase

- **Start Condition:** Rover must autonomously exit the start area for points.
- **Navigation:**
  - Task: Navigate to five placards using a pre-provided schematic.
  - **Points:** 6 points per placard imaged and relayed to judges.
- **Rules:**
  - No manual control once rover begins moving.
  - Interventions move to non-autonomous phase.

### Non-Autonomous Phase

- Teams can take manual control anytime, for further exploration or troubleshooting which ends the ability to gather points in the autonomous phase.

### Exploratory Mapping

- **Objective:** Locate four 100x100x100mm cubes (red, green, blue, white).
- **Points:**
  - 5 points for each cube located within 300mm accuracy.
  - 2 points if within 600mm.
  - Double points if reported during autonomous phase.

#### Data and Mapping Restrictions

- All mapping data must be gathered during the task; no prior arena knowledge allowed.

#### Presentation

- **Autonomous Navigation Design:**
  - Discuss the autonomous system's design, advantages, and limitations.
- **Mapping System Design:**
  - Explain mapping navigation methods.
  - Justify autonomy level, map format, and feature choices.
- **Map Visualisation:**
  - Present arena map, judged for coverage, completeness, resolution, and accuracy.
- **Points:** Up to 25 points for the quality of the presentation.

### Scoring and Penalties

- Points for navigation success, cube location accuracy, and presentation.
- Penalties for autonomous phase collisions or exiting arena requiring E-STOP activation.

This task emphasises autonomous operation, navigation, and mapping, with a focus on practical application of robotics in space exploration scenarios.
