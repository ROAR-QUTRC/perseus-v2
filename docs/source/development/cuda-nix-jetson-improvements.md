# CUDA/Nix/Jetson Workflow Improvements

This document outlines planned improvements for faster iteration, CUDA version robustness, and CPU-only operation modes for the Perseus-v2 project running ROS 2 with Nix on Jetson Orin.

## Background

### Current Architecture

- **Nix CUDA**: Compilation environment with nvcc, headers, and libraries from `/nix/store`
- **Host CUDA**: Runtime via `nixcuda` wrapper for GPU driver passthrough
- **Jetson targets**: big-brain, medium-brain (aarch64-linux with JetPack CUDA 12.x)
- **Autonomy stack**: SLAM (slam_toolbox) + Nav2 - currently CPU-only

### Key Files

| File                            | Purpose                                       |
| ------------------------------- | --------------------------------------------- |
| `flake.nix`                     | Nix configuration, CUDA packages, shell hooks |
| `software/ros_ws/overlay.nix`   | nixcuda wrapper, ROS package overlays         |
| `software/scripts/cuda-test.sh` | Existing CUDA validation script               |
| `software/scripts/cuda-demo.sh` | CUDA vs CPU benchmark demo                    |

---

## Part 1: Faster Iteration

### 1.1 Create Project CLAUDE.md

Create `/home/dingo/perseus-v2/CLAUDE.md` with project-specific context:

````markdown
# Perseus-v2 Project Context

## Architecture

- ROS2 Jazzy on Nix with CUDA support
- Jetson Orin targets: big-brain, medium-brain (aarch64-linux)
- Dev machines: x86_64-linux with optional GPU

## CUDA Environment

The project uses a two-layer CUDA approach:

1. **Nix CUDA**: Portable compilation (nvcc, headers, libs in /nix/store)
2. **Host CUDA**: Runtime via `nixcuda` wrapper for GPU access

### Running CUDA Programs

```bash
# Compile with Nix CUDA
nvcc -o program program.cu

# Run with host GPU libraries
nixcuda ./program
```

## Key Commands

- `./software/scripts/cuda-test.sh` - Verify CUDA environment
- `./software/scripts/cuda-demo.sh` - Visual CUDA vs CPU demo
- `./software/scripts/cuda-version-check.sh` - Version compatibility check
- `./software/scripts/cuda-mode.sh` - Enable/disable CUDA
- `nix develop` - Enter dev shell with all dependencies

## Autonomy Stack (CPU-only currently)

- SLAM: slam_toolbox with CeresSolver (CPU)
- Navigation: Full Nav2 stack (CPU)
- Teleoperation: joy_node -> xbox_controller -> cmd_vel_mux -> /cmd_vel

## Common Issues

1. **CUDA version mismatch**: Check `nvcc --version` vs `nvidia-smi` CUDA version
2. **Missing GPU libraries**: Use `nixcuda` wrapper, not direct execution
3. **TF staleness**: Check robot_state_publisher is running
4. **GPU not detected**: Ensure you're using `nixcuda` for CUDA programs

````

### 1.2 Add CUDA Diagnostics Skill

Create `/home/dingo/perseus-v2/.claude/commands/cuda-check.md`:

```markdown
---
description: Run CUDA environment diagnostics
---

# CUDA Check

Run the following diagnostic sequence:

1. Check if in Nix dev shell:
```bash
echo "CUDA_PATH: ${CUDA_PATH:-NOT SET}"
echo "CUDA_HOME: ${CUDA_HOME:-NOT SET}"
which nvcc || echo "nvcc not found"
````

2. Check Nix CUDA version:

```bash
nvcc --version 2>/dev/null | grep release || echo "No nvcc"
```

3. Check host GPU and driver:

```bash
nvidia-smi --query-gpu=name,driver_version,compute_cap --format=csv 2>/dev/null || echo "No GPU/driver"
```

4. Check Jetson-specific paths (if on aarch64):

```bash
ls -la /usr/lib/aarch64-linux-gnu/tegra/ 2>/dev/null | head -5
ls -la /usr/local/cuda/lib64/ 2>/dev/null | head -5
```

5. Run version compatibility check:

```bash
./software/scripts/cuda-version-check.sh
```

6. Run quick compilation test:

```bash
./software/scripts/cuda-test.sh 2>&1 | tail -30
```

Report findings with recommendations for any mismatches.

### 1.3 Add Version Check Script

Create `/home/dingo/perseus-v2/software/scripts/cuda-version-check.sh`:

```bash
#!/usr/bin/env bash
# CUDA Version Compatibility Checker for Nix/Jetson environments

set -uo pipefail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}=== CUDA Version Compatibility Check ===${NC}"
echo ""

# 1. Get Nix CUDA version (compile-time)
NIX_CUDA_VERSION=""
if command -v nvcc &>/dev/null; then
    NIX_CUDA_VERSION=$(nvcc --version | grep "release" | sed 's/.*release \([0-9.]*\),.*/\1/')
    echo -e "Nix CUDA (nvcc):      ${GREEN}${NIX_CUDA_VERSION}${NC}"
else
    echo -e "Nix CUDA (nvcc):      ${RED}NOT FOUND${NC}"
    echo "  Hint: Are you in the Nix dev shell? Run 'nix develop'"
fi

# 2. Get Host CUDA version (runtime)
HOST_CUDA_VERSION=""
if command -v nvidia-smi &>/dev/null; then
    HOST_CUDA_VERSION=$(nvidia-smi 2>/dev/null | grep "CUDA Version" | sed 's/.*CUDA Version: \([0-9.]*\).*/\1/')
    if [[ -n "$HOST_CUDA_VERSION" ]]; then
        echo -e "Host CUDA (driver):   ${GREEN}${HOST_CUDA_VERSION}${NC}"
    fi
fi

# Check for Jetson-specific CUDA
if [[ -z "$HOST_CUDA_VERSION" ]]; then
    if [[ -f /usr/local/cuda/version.txt ]]; then
        HOST_CUDA_VERSION=$(cat /usr/local/cuda/version.txt | grep -oP 'CUDA Version \K[0-9.]+' || true)
        echo -e "Host CUDA (Jetson):   ${GREEN}${HOST_CUDA_VERSION}${NC}"
    elif [[ -f /etc/nv_tegra_release ]]; then
        echo -e "Host CUDA:            ${YELLOW}Jetson detected, checking...${NC}"
        # Try to find CUDA version from JetPack
        if [[ -d /usr/local/cuda ]]; then
            HOST_CUDA_VERSION=$(readlink /usr/local/cuda | grep -oP 'cuda-\K[0-9.]+' || echo "")
            if [[ -n "$HOST_CUDA_VERSION" ]]; then
                echo -e "Host CUDA (JetPack):  ${GREEN}${HOST_CUDA_VERSION}${NC}"
            fi
        fi
    else
        echo -e "Host CUDA:            ${YELLOW}NOT DETECTED${NC}"
    fi
fi

# 3. Check for Tegra libraries (Jetson)
echo ""
JETSON_MODE=false
if [[ -d /usr/lib/aarch64-linux-gnu/tegra ]]; then
    echo -e "Jetson Tegra libs:    ${GREEN}PRESENT${NC}"
    JETSON_MODE=true
elif [[ -f /etc/nv_tegra_release ]]; then
    echo -e "Jetson Tegra libs:    ${YELLOW}EXPECTED BUT NOT FOUND${NC}"
    JETSON_MODE=true
else
    echo -e "Jetson Tegra libs:    ${CYAN}N/A (not Jetson)${NC}"
fi

# 4. Version compatibility check
echo ""
echo -e "${CYAN}=== Compatibility Analysis ===${NC}"

if [[ -n "$NIX_CUDA_VERSION" && -n "$HOST_CUDA_VERSION" ]]; then
    NIX_MAJOR=$(echo "$NIX_CUDA_VERSION" | cut -d. -f1)
    HOST_MAJOR=$(echo "$HOST_CUDA_VERSION" | cut -d. -f1)
    NIX_MINOR=$(echo "$NIX_CUDA_VERSION" | cut -d. -f2)
    HOST_MINOR=$(echo "$HOST_CUDA_VERSION" | cut -d. -f2)

    if [[ "$NIX_MAJOR" == "$HOST_MAJOR" ]]; then
        if [[ "$NIX_MINOR" == "$HOST_MINOR" ]]; then
            echo -e "${GREEN}COMPATIBLE: Versions match (CUDA $NIX_CUDA_VERSION)${NC}"
        else
            echo -e "${GREEN}COMPATIBLE: Major versions match (CUDA $NIX_MAJOR.x)${NC}"
            echo "  Nix: $NIX_CUDA_VERSION, Host: $HOST_CUDA_VERSION"
        fi
    elif [[ "$HOST_MAJOR" -gt "$NIX_MAJOR" ]]; then
        echo -e "${YELLOW}WARNING: Host CUDA ($HOST_CUDA_VERSION) newer than Nix CUDA ($NIX_CUDA_VERSION)${NC}"
        echo "  - Programs compiled with Nix CUDA should still run"
        echo "  - Some newer GPU features may not be available at compile time"
        echo "  - This is usually fine for forward compatibility"
    else
        echo -e "${RED}POTENTIAL ISSUE: Host CUDA ($HOST_CUDA_VERSION) older than Nix CUDA ($NIX_CUDA_VERSION)${NC}"
        echo "  - Runtime may fail if newer CUDA features are used"
        echo "  - Consider updating JetPack or checking nixpkgs CUDA version"
    fi
elif [[ -z "$NIX_CUDA_VERSION" ]]; then
    echo -e "${YELLOW}Cannot check: Nix CUDA not available${NC}"
    echo "  Run 'nix develop' to enter the dev shell"
elif [[ -z "$HOST_CUDA_VERSION" ]]; then
    echo -e "${YELLOW}Cannot check: Host CUDA not detected${NC}"
    echo "  GPU driver may not be installed or accessible"
fi

# 5. Environment variables
echo ""
echo -e "${CYAN}=== Environment Variables ===${NC}"
echo "CUDA_PATH:            ${CUDA_PATH:-NOT SET}"
echo "CUDA_HOME:            ${CUDA_HOME:-NOT SET}"
echo "CUDA_VISIBLE_DEVICES: ${CUDA_VISIBLE_DEVICES:-NOT SET (all GPUs visible)}"

CUDA_IN_PATH=$(echo "${LD_LIBRARY_PATH:-}" | grep -c cuda || echo 0)
echo "LD_LIBRARY_PATH:      $CUDA_IN_PATH CUDA path(s)"

# 6. Recommendations
echo ""
echo -e "${CYAN}=== Recommendations ===${NC}"
if $JETSON_MODE; then
    echo "1. Use 'nixcuda' wrapper for all CUDA binaries"
    echo "2. Ensure LD_LIBRARY_PATH includes Tegra paths"
    echo "3. Run './software/scripts/cuda-test.sh' for full validation"
else
    echo "1. Use 'nixcuda' wrapper for GPU programs"
    echo "2. Run './software/scripts/cuda-test.sh' for full validation"
fi

# 7. Quick GPU check
echo ""
if command -v nvidia-smi &>/dev/null; then
    echo -e "${CYAN}=== GPU Status ===${NC}"
    nvidia-smi --query-gpu=name,memory.total,compute_cap --format=csv,noheader 2>/dev/null || echo "GPU query failed"
fi
````

---

## Part 2: CUDA Version Robustness

### 2.1 Enhance nixcuda Wrapper

Modify `/home/dingo/perseus-v2/software/ros_ws/overlay.nix` lines 17-25.

**Current implementation:**

```nix
nixcuda-script = prev.writeShellScriptBin "nixcuda" ''
  # Add Jetson/Tegra CUDA library paths if they exist
  for cuda_path in /usr/lib/aarch64-linux-gnu/tegra /usr/local/cuda/lib64; do
    if [ -d "$cuda_path" ]; then
      export LD_LIBRARY_PATH="$cuda_path''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
  done
  exec ${prev.nix-gl-host}/bin/nixglhost "$@"
'';
```

**Enhanced implementation with version detection:**

```nix
nixcuda-script = prev.writeShellScriptBin "nixcuda" ''
  # Detect host CUDA version for dynamic path resolution
  HOST_CUDA=""
  if command -v nvidia-smi &>/dev/null; then
    HOST_CUDA=$(nvidia-smi 2>/dev/null | grep -oP 'CUDA Version: \K[0-9]+' || true)
  elif [[ -f /usr/local/cuda/version.txt ]]; then
    HOST_CUDA=$(grep -oP 'CUDA Version \K[0-9]+' /usr/local/cuda/version.txt 2>/dev/null || true)
  fi

  # Add Jetson/Tegra CUDA library paths if they exist
  # Include versioned path based on detected host CUDA
  for cuda_path in \
    /usr/lib/aarch64-linux-gnu/tegra \
    /usr/local/cuda/lib64 \
    ''${HOST_CUDA:+/usr/local/cuda-$HOST_CUDA/lib64}; do
    if [[ -n "$cuda_path" ]] && [[ -d "$cuda_path" ]]; then
      export LD_LIBRARY_PATH="$cuda_path''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
  done

  # Set default CUDA_VISIBLE_DEVICES if not already set
  # This allows cuda-mode.sh to control GPU visibility
  : "''${CUDA_VISIBLE_DEVICES:=all}"
  export CUDA_VISIBLE_DEVICES

  exec ${prev.nix-gl-host}/bin/nixglhost "$@"
'';
```

### 2.2 Version Strategy

**Approach: Runtime detection only** (no version pinning in flake.nix)

The enhanced `nixcuda` wrapper detects and adapts to the host CUDA version at runtime. This avoids:

- Maintaining version pins that may drift
- Rebuilding when CUDA versions change
- Conflicts between different Jetson devices with different JetPack versions

If explicit pinning becomes necessary later, add to `flake.nix`:

```nix
cudaPackages = prev.cudaPackages_12;  # Match JetPack CUDA 12
```

---

## Part 3: CUDA Toggle for CPU-Only Operation

### 3.1 Create CUDA Mode Script

Create `/home/dingo/perseus-v2/software/scripts/cuda-mode.sh`:

```bash
#!/usr/bin/env bash
# CUDA Mode Toggle Script for Perseus-v2
# Usage: source cuda-mode.sh [enable|disable|status]
#
# This script controls CUDA/GPU availability for the current shell session.
# When disabled, CUDA_VISIBLE_DEVICES="" forces all CUDA programs to fall back to CPU.

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# Persistent mode file (survives shell restarts)
CUDA_MODE_FILE="${HOME}/.perseus_cuda_mode"

cuda_enable() {
    echo "enabled" > "$CUDA_MODE_FILE"
    export PERSEUS_CUDA_ENABLED=1
    export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES_SAVED:-all}"
    unset CUDA_DISABLED

    echo -e "${GREEN}CUDA mode ENABLED${NC}"
    echo "  CUDA_VISIBLE_DEVICES=$CUDA_VISIBLE_DEVICES"
    echo ""
    echo "GPU programs will use available CUDA devices."
    echo "Use 'nixcuda <program>' to run with GPU support."
}

cuda_disable() {
    echo "disabled" > "$CUDA_MODE_FILE"
    export PERSEUS_CUDA_ENABLED=0

    # Save current value before clearing
    if [[ -n "${CUDA_VISIBLE_DEVICES:-}" && "${CUDA_VISIBLE_DEVICES}" != "" ]]; then
        export CUDA_VISIBLE_DEVICES_SAVED="$CUDA_VISIBLE_DEVICES"
    fi

    export CUDA_VISIBLE_DEVICES=""
    export CUDA_DISABLED=1

    echo -e "${YELLOW}CUDA mode DISABLED${NC}"
    echo "  CUDA_VISIBLE_DEVICES=\"\" (empty - forces CPU fallback)"
    echo ""
    echo "All CUDA programs will run in CPU-only mode."
    echo "This is useful for:"
    echo "  - Running teleoperation + SLAM + Nav2 without GPU"
    echo "  - Testing CPU fallback paths"
    echo "  - Debugging GPU-related issues"
}

cuda_status() {
    echo -e "${CYAN}=== CUDA Mode Status ===${NC}"
    echo ""

    # Check environment
    if [[ "${PERSEUS_CUDA_ENABLED:-}" == "1" ]]; then
        echo -e "Mode:                 ${GREEN}ENABLED${NC}"
    elif [[ "${CUDA_DISABLED:-}" == "1" ]] || [[ "${CUDA_VISIBLE_DEVICES:-x}" == "" ]]; then
        echo -e "Mode:                 ${YELLOW}DISABLED${NC}"
    else
        echo -e "Mode:                 ${CYAN}AUTO (hardware detection)${NC}"
    fi

    echo "CUDA_VISIBLE_DEVICES: ${CUDA_VISIBLE_DEVICES:-not set}"
    echo "PERSEUS_CUDA_ENABLED: ${PERSEUS_CUDA_ENABLED:-not set}"

    # Check if GPU is actually available
    echo ""
    if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
        echo -e "GPU Hardware:         ${GREEN}AVAILABLE${NC}"
        nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1
    else
        echo -e "GPU Hardware:         ${YELLOW}NOT AVAILABLE${NC}"
    fi

    # Check persistent setting
    if [[ -f "$CUDA_MODE_FILE" ]]; then
        echo ""
        echo "Persistent setting:   $(cat "$CUDA_MODE_FILE")"
    fi
}

cuda_help() {
    echo "Usage: source cuda-mode.sh [command]"
    echo ""
    echo "Commands:"
    echo "  enable, on     Enable CUDA/GPU (restore CUDA_VISIBLE_DEVICES)"
    echo "  disable, off   Disable CUDA (set CUDA_VISIBLE_DEVICES=\"\")"
    echo "  cpu            Alias for disable"
    echo "  status         Show current CUDA mode (default)"
    echo "  help           Show this help"
    echo ""
    echo "Examples:"
    echo "  source cuda-mode.sh disable"
    echo "  ros2 launch autonomy perseus_mapping.launch.py"
    echo "  source cuda-mode.sh enable"
}

# Main dispatch
case "${1:-status}" in
    enable|on)
        cuda_enable
        ;;
    disable|off|cpu)
        cuda_disable
        ;;
    status)
        cuda_status
        ;;
    help|-h|--help)
        cuda_help
        ;;
    *)
        echo "Unknown command: $1"
        cuda_help
        exit 1
        ;;
esac
```

### 3.2 Toggle Strategy

**Environment variable only** - no launch file modifications needed.

The `CUDA_VISIBLE_DEVICES=""` approach:

- Works with any CUDA program (Python, C++, etc.)
- No code changes required in ROS nodes
- Universal fallback mechanism supported by CUDA runtime
- Easy to toggle per-session or per-terminal

### 3.3 Usage Examples

```bash
# Enter Nix dev shell
nix develop

# Check current CUDA status
source software/scripts/cuda-mode.sh status

# Disable CUDA for CPU-only operation
source software/scripts/cuda-mode.sh disable

# Run teleoperation with SLAM and Nav2 (all CPU-based)
ros2 launch autonomy perseus_mapping.launch.py &
ros2 launch autonomy perseus_nav_bringup.launch.py &
ros2 launch input_devices xbox_controller.launch.py

# Later, re-enable CUDA for ML inference
source software/scripts/cuda-mode.sh enable
nixcuda python3 inference_node.py
```

---

## Implementation Checklist

### Files to Create

- [ ] `/home/dingo/perseus-v2/CLAUDE.md` - Project context
- [ ] `/home/dingo/perseus-v2/.claude/commands/cuda-check.md` - Diagnostics skill
- [ ] `/home/dingo/perseus-v2/software/scripts/cuda-version-check.sh` - Version checker
- [ ] `/home/dingo/perseus-v2/software/scripts/cuda-mode.sh` - CUDA toggle

### Files to Modify

- [ ] `/home/dingo/perseus-v2/software/ros_ws/overlay.nix` - Enhance nixcuda wrapper

### Verification Steps

1. **Test version checker:**

   ```bash
   chmod +x software/scripts/cuda-version-check.sh
   ./software/scripts/cuda-version-check.sh
   ```

2. **Test CUDA toggle:**

   ```bash
   chmod +x software/scripts/cuda-mode.sh
   source software/scripts/cuda-mode.sh status
   source software/scripts/cuda-mode.sh disable
   # Verify CUDA_VISIBLE_DEVICES is empty
   echo $CUDA_VISIBLE_DEVICES
   source software/scripts/cuda-mode.sh enable
   ```

3. **Test CPU-only autonomy:**

   ```bash
   source software/scripts/cuda-mode.sh disable
   ros2 launch autonomy perseus_mapping.launch.py
   # Verify SLAM runs without GPU errors
   ```

4. **Test enhanced nixcuda (after modification):**
   ```bash
   nix develop
   nixcuda ./software/scripts/cuda-test.sh
   ```

---

## Notes

- Current SLAM (slam_toolbox) and Nav2 are already CPU-only
- CUDA is used for ONNX Runtime inference workloads
- The toggle prevents future GPU-accelerated nodes from claiming GPU resources
- Jetson Orin has CUDA 12.x via JetPack; Nix CUDA depends on nixpkgs pin
- The `nixcuda` wrapper handles the Nix/host CUDA library mismatch
