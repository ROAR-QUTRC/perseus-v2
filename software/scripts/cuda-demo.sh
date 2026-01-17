#!/usr/bin/env bash

# CUDA Matrix Multiplication Demo - Side-by-Side Race Mode
# Visual demonstration of CUDA acceleration vs CPU computation

set -uo pipefail

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
DIM='\033[2m'
NC='\033[0m' # No Color

# ANSI cursor control
HIDE_CURSOR='\033[?25l'
SHOW_CURSOR='\033[?25h'
CLEAR_LINE='\033[K'

# Default parameters
MATRIX_SIZE=1024
SEQUENTIAL_MODE=false

# Progress bar characters
BAR_FILLED='█'
BAR_EMPTY='░'
BAR_WIDTH=30

print_header() {
    echo -e "${BLUE}${BOLD}=== $1 ===${NC}"
}

print_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[FAIL]${NC} $1"
}

print_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Visual demonstration of CUDA acceleration using Matrix Multiplication"
    echo "Default mode shows CPU and CUDA racing side-by-side in real-time"
    echo ""
    echo "Options:"
    echo "  -n, --size N        Matrix size NxN (default: $MATRIX_SIZE)"
    echo "  -s, --sequential    Run sequentially instead of side-by-side"
    echo "  --help              Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                    # Side-by-side race (default)"
    echo "  $0 -n 768             # Larger matrices (more dramatic difference)"
    echo "  $0 -n 256             # Smaller matrices (faster demo)"
    echo "  $0 -s                 # Sequential mode"
}

# Draw a progress bar
draw_progress_bar() {
    local percent=$1
    local color=$2
    local filled=$((percent * BAR_WIDTH / 100))
    local empty=$((BAR_WIDTH - filled))

    printf "${color}"
    for ((i=0; i<filled; i++)); do printf "%s" "$BAR_FILLED"; done
    printf "${DIM}"
    for ((i=0; i<empty; i++)); do printf "%s" "$BAR_EMPTY"; done
    printf "${NC}"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -n|--size)
            MATRIX_SIZE="$2"
            shift 2
            ;;
        -s|--sequential)
            SEQUENTIAL_MODE=true
            shift
            ;;
        --help)
            print_usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            print_usage
            exit 1
            ;;
    esac
done

# Create temp directory for compilation
TEMP_DIR=$(mktemp -d)
trap "rm -rf $TEMP_DIR; printf '${SHOW_CURSOR}' 2>/dev/null || true" EXIT

CPU_SOURCE="$SCRIPT_DIR/matrix_cpu.c"
CUDA_SOURCE="$SCRIPT_DIR/matrix_cuda.cu"
CPU_BINARY="$TEMP_DIR/matrix_cpu"
CUDA_BINARY="$TEMP_DIR/matrix_cuda"

# Check if source files exist
if [[ ! -f "$CPU_SOURCE" ]]; then
    print_error "CPU source not found: $CPU_SOURCE"
    exit 1
fi

if [[ ! -f "$CUDA_SOURCE" ]]; then
    print_error "CUDA source not found: $CUDA_SOURCE"
    exit 1
fi

echo ""
echo -e "${MAGENTA}${BOLD}"
echo "   __  __       _        _        __  __       _ _   "
echo "  |  \/  | __ _| |_ _ __(_)_  __ |  \/  |_   _| | |_ "
echo "  | |\/| |/ _\` | __| '__| \ \/ / | |\/| | | | | | __|"
echo "  | |  | | (_| | |_| |  | |>  <  | |  | | |_| | | |_ "
echo "  |_|  |_|\__,_|\__|_|  |_/_/\_\ |_|  |_|\__,_|_|\__|"
echo ""
echo "         CUDA vs CPU Performance Demo"
echo -e "${NC}"
echo ""

# Calculate operations
OPS=$(awk "BEGIN {printf \"%.1f\", ($MATRIX_SIZE * $MATRIX_SIZE * $MATRIX_SIZE) / 1000000}")
echo -e "${DIM}Matrix size: ${MATRIX_SIZE}x${MATRIX_SIZE} (${OPS}M operations)${NC}"
echo ""

# Compile CPU version
print_header "Compiling"
if gcc -O3 -o "$CPU_BINARY" "$CPU_SOURCE" -lm 2>/dev/null; then
    print_success "CPU version compiled"
else
    print_error "Failed to compile CPU version"
    exit 1
fi

# Check for CUDA toolkit and compile CUDA version
CUDA_AVAILABLE=false
GPU_AVAILABLE=false

if command -v nvcc &>/dev/null; then
    if nvcc -O3 -o "$CUDA_BINARY" "$CUDA_SOURCE" 2>/dev/null; then
        CUDA_AVAILABLE=true
        print_success "CUDA version compiled"

        # Check for GPU
        if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
            GPU_AVAILABLE=true
        fi
    else
        print_warning "CUDA compilation failed - will run CPU-only demo"
    fi
else
    print_warning "nvcc not found - will run CPU-only demo"
fi

echo ""

# Sequential mode (original behavior)
if $SEQUENTIAL_MODE || ! $CUDA_AVAILABLE; then
    ARGS="-n $MATRIX_SIZE"

    print_header "Running CPU Version"
    echo ""
    CPU_OUTPUT=$("$CPU_BINARY" $ARGS)
    echo "$CPU_OUTPUT" | grep -v "TIME_MS:" | grep -v "CHECKSUM:"
    CPU_TIME=$(echo "$CPU_OUTPUT" | grep "TIME_MS:" | cut -d: -f2)
    echo ""
    echo -e "CPU Time: ${YELLOW}${BOLD}${CPU_TIME} ms${NC}"
    echo ""

    if $CUDA_AVAILABLE; then
        print_header "Running CUDA Version"
        echo ""

        if $GPU_AVAILABLE && command -v nixcuda &>/dev/null; then
            CUDA_OUTPUT=$(nixcuda "$CUDA_BINARY" $ARGS 2>/dev/null || echo "GPU_USED:0")
        else
            CUDA_OUTPUT=$("$CUDA_BINARY" $ARGS 2>/dev/null || echo "GPU_USED:0")
        fi

        echo "$CUDA_OUTPUT" | grep -v "TIME_MS:" | grep -v "GPU_USED:" | grep -v "CHECKSUM:"
        CUDA_TIME=$(echo "$CUDA_OUTPUT" | grep "TIME_MS:" | cut -d: -f2)
        GPU_USED=$(echo "$CUDA_OUTPUT" | grep "GPU_USED:" | cut -d: -f2)

        echo ""
        if [[ "$GPU_USED" == "1" ]]; then
            echo -e "CUDA Time: ${GREEN}${BOLD}${CUDA_TIME} ms${NC}"
            SPEEDUP=$(awk "BEGIN {printf \"%.1f\", $CPU_TIME / $CUDA_TIME}")
            echo -e "Speedup: ${GREEN}${BOLD}${SPEEDUP}x${NC}"
        else
            echo -e "CUDA (CPU fallback) Time: ${YELLOW}${BOLD}${CUDA_TIME} ms${NC}"
        fi
    fi

    echo ""
    exit 0
fi

# ============================================================================
# Side-by-side race mode
# ============================================================================
print_header "Side-by-Side Race Mode"
echo ""

# Files for storing output
CPU_OUTPUT_FILE="$TEMP_DIR/cpu_output.txt"
CUDA_OUTPUT_FILE="$TEMP_DIR/cuda_output.txt"

# Initialize output files
touch "$CPU_OUTPUT_FILE" "$CUDA_OUTPUT_FILE"

# Print header row
printf "  ${YELLOW}${BOLD}%-$((BAR_WIDTH + 15))s${NC}" "CPU"
printf "  ${GREEN}${BOLD}%-$((BAR_WIDTH + 15))s${NC}\n" "CUDA"

# Print separator
printf "  ${DIM}"
for ((i=0; i<BAR_WIDTH + 15; i++)); do printf "─"; done
printf "  "
for ((i=0; i<BAR_WIDTH + 15; i++)); do printf "─"; done
printf "${NC}\n"

# Hide cursor
printf "${HIDE_CURSOR}"

# Print initial progress bars
printf "  "
draw_progress_bar 0 "$YELLOW"
printf "   0%%   0.0ms"
printf "  "
draw_progress_bar 0 "$GREEN"
printf "   0%%   0.0ms\n"

# Status line
printf "  ${DIM}Starting...${NC}\n"

# Move cursor back to progress line
printf "\033[2A"

# Start both processes
"$CPU_BINARY" -n "$MATRIX_SIZE" -s > "$CPU_OUTPUT_FILE" 2>/dev/null &
CPU_PID=$!

if $GPU_AVAILABLE && command -v nixcuda &>/dev/null; then
    nixcuda "$CUDA_BINARY" -n "$MATRIX_SIZE" -s > "$CUDA_OUTPUT_FILE" 2>/dev/null &
else
    "$CUDA_BINARY" -n "$MATRIX_SIZE" -s > "$CUDA_OUTPUT_FILE" 2>/dev/null &
fi
CUDA_PID=$!

# Track state
cpu_percent=0
cuda_percent=0
cpu_time="0.0"
cuda_time="0.0"
cpu_final_time=""
cuda_final_time=""
gpu_used="0"
cpu_line=0
cuda_line=0

# Wait for processes to start writing
sleep 0.05

# Main loop - poll output files and update display
while true; do
    updated=false

    # Read CPU lines
    cpu_line_count=$(wc -l < "$CPU_OUTPUT_FILE" 2>/dev/null || echo 0)
    while [[ $cpu_line -lt $cpu_line_count ]]; do
        line=$(sed -n "$((cpu_line + 1))p" "$CPU_OUTPUT_FILE")

        if [[ "$line" == PROGRESS:* ]]; then
            # Parse: PROGRESS:percent|elapsed_ms
            data="${line#PROGRESS:}"
            cpu_percent="${data%|*}"
            cpu_time="${data#*|}"
            updated=true
        elif [[ "$line" == TIME_MS:* ]]; then
            cpu_final_time="${line#TIME_MS:}"
        fi
        ((cpu_line++)) || true
    done

    # Read CUDA lines
    cuda_line_count=$(wc -l < "$CUDA_OUTPUT_FILE" 2>/dev/null || echo 0)
    while [[ $cuda_line -lt $cuda_line_count ]]; do
        line=$(sed -n "$((cuda_line + 1))p" "$CUDA_OUTPUT_FILE")

        if [[ "$line" == PROGRESS:* ]]; then
            data="${line#PROGRESS:}"
            cuda_percent="${data%|*}"
            cuda_time="${data#*|}"
            updated=true
        elif [[ "$line" == TIME_MS:* ]]; then
            cuda_final_time="${line#TIME_MS:}"
        elif [[ "$line" == GPU_USED:* ]]; then
            gpu_used="${line#GPU_USED:}"
        fi
        ((cuda_line++)) || true
    done

    # Update display if changed
    if $updated; then
        printf "\r  "
        draw_progress_bar "$cpu_percent" "$YELLOW"
        printf " %3d%% %7.1fms" "$cpu_percent" "$cpu_time"
        printf "  "
        draw_progress_bar "$cuda_percent" "$GREEN"
        printf " %3d%% %7.1fms" "$cuda_percent" "$cuda_time"
    fi

    # Check completion
    if [[ -n "$cpu_final_time" && -n "$cuda_final_time" ]]; then
        break
    fi

    # Check if processes are still running
    cpu_running=true
    cuda_running=true
    kill -0 $CPU_PID 2>/dev/null || cpu_running=false
    kill -0 $CUDA_PID 2>/dev/null || cuda_running=false

    if ! $cpu_running && ! $cuda_running; then
        # Give files time to flush
        sleep 0.05
        # Continue to read remaining output
        if [[ -n "$cpu_final_time" && -n "$cuda_final_time" ]]; then
            break
        fi
    fi

    sleep 0.02
done

# Wait for processes
wait $CPU_PID 2>/dev/null || true
wait $CUDA_PID 2>/dev/null || true

# Final update with 100%
printf "\r  "
draw_progress_bar 100 "$YELLOW"
printf " 100%% %7.1fms" "$cpu_final_time"
printf "  "
draw_progress_bar 100 "$GREEN"
printf " 100%% %7.1fms\n" "$cuda_final_time"

# Move past status line
printf "\n"

# Show cursor
printf "${SHOW_CURSOR}"

# Results
echo ""
print_header "Results"
echo ""

[[ -z "$cpu_final_time" ]] && cpu_final_time="N/A"
[[ -z "$cuda_final_time" ]] && cuda_final_time="N/A"

printf "  ${YELLOW}CPU Time:  ${BOLD}%s ms${NC}\n" "$cpu_final_time"

if [[ "$gpu_used" == "1" ]]; then
    printf "  ${GREEN}CUDA Time: ${BOLD}%s ms${NC} (GPU accelerated)\n" "$cuda_final_time"

    if [[ "$cpu_final_time" != "N/A" && "$cuda_final_time" != "N/A" ]]; then
        speedup=$(awk "BEGIN {printf \"%.1f\", $cpu_final_time / $cuda_final_time}")
        echo ""
        printf "  ${GREEN}${BOLD}>>> Speedup: %sx faster with CUDA! <<<${NC}\n" "$speedup"
    fi
else
    printf "  ${YELLOW}CUDA Time: ${BOLD}%s ms${NC} (CPU fallback - no GPU)\n" "$cuda_final_time"
    echo ""
    printf "  ${CYAN}Run with nixcuda or on a GPU system for acceleration${NC}\n"
fi

echo ""
printf "${DIM}Tip: Try -n 768 or -n 1024 for more dramatic speed differences${NC}\n"
echo ""
