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
autonomy-diagnostics-tool
```

This section provides the definitive reference for Perseus autonomy operations. It serves as both a repeatable bring-up procedure and a contract specification—changes to items marked as **contracts** require coordination with the autonomy team before implementation.

**2026 Team Members**

- Nigel
- Natalia Hoyos Gonzalez
- Jasper
- Lachlan Ikeguchi

**Document Structure**

| Page | Description |
|------|-------------|
| [Quick Start](autonomy-quick-start.md) | Copy-paste commands for bring-up |
| [Architecture](autonomy-architecture.md) | System design, TF tree, data flow |
| [Interface Contracts](autonomy-interface-contracts.md) | Topics, frames, and hardware interfaces that must not change |
| [Physical Constants](autonomy-physical-constants.md) | Geometry and sensor specs |
| [Navigation Tuning](autonomy-navigation-tuning.md) | Nav2, AMCL, and SLAM parameters |
| [Launch Reference](autonomy-launch-reference.md) | Detailed launch file documentation |
| [Pre-Flight Checklist](autonomy-preflight-checklist.md) | Test-day verification steps |
| [Troubleshooting](autonomy-troubleshooting.md) | Common issues and debug commands |
| [Diagnostics Tool](autonomy-diagnostics-tool.md) | Real-time TUI for system health monitoring |

**Who Should Read This**

| Subteam | Start Here |
|---------|------------|
| Software | All pages |
| Electrical | [Interface Contracts](autonomy-interface-contracts.md), [Physical Constants](autonomy-physical-constants.md) |
| Mechanical | [Physical Constants](autonomy-physical-constants.md) |
| All members | [Quick Start](autonomy-quick-start.md), [Architecture](autonomy-architecture.md) |

**When to Update**

- **Before** any mechanical, electrical, or software change affecting sensors, drivetrain, or frames
- **After** test days with observed discrepancies
- **After** any calibration or tuning session
