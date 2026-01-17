#!/usr/bin/env bash

# CUDA environment test script
# Tests the status of Nix-managed CUDA support

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
  echo -e "${BLUE}=== $1 ===${NC}"
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

print_info() {
  echo -e "     $1"
}

# Track overall status
CUDA_TOOLKIT_OK=false
CUDA_RUNTIME_OK=false
GPU_AVAILABLE=false

echo ""
print_header "Nix CUDA Environment Test"
echo ""

# Test 1: Check CUDA toolkit (nvcc)
print_header "CUDA Toolkit (Nix-managed)"

if command -v nvcc &>/dev/null; then
  CUDA_TOOLKIT_OK=true
  NVCC_PATH=$(which nvcc)
  NVCC_VERSION=$(nvcc --version | grep "release" | sed 's/.*release //' | sed 's/,.*//')
  print_success "nvcc found: $NVCC_PATH"
  print_info "CUDA Version: $NVCC_VERSION"

  # Check if it's from Nix store
  if [[ $NVCC_PATH == /nix/store/* ]]; then
    print_success "nvcc is Nix-managed (portable)"
  else
    print_warning "nvcc is not from Nix store"
  fi
else
  print_error "nvcc not found"
  print_info "Make sure you're in the Nix dev shell: nix develop"
fi

echo ""

# Test 2: Check CUDA environment variables
print_header "CUDA Environment Variables"

if [[ -n ${CUDA_PATH:-} ]]; then
  print_success "CUDA_PATH: $CUDA_PATH"
else
  print_warning "CUDA_PATH not set"
fi

if [[ -n ${CUDA_HOME:-} ]]; then
  print_success "CUDA_HOME: $CUDA_HOME"
else
  print_warning "CUDA_HOME not set"
fi

echo ""

# Test 3: Check nixcuda wrapper
print_header "CUDA Runtime Wrapper"

if command -v nixcuda &>/dev/null; then
  print_success "nixcuda wrapper available"
  print_info "Use 'nixcuda <program>' to run CUDA binaries with GPU access"
else
  print_warning "nixcuda wrapper not found"
fi

if command -v nixglhost &>/dev/null; then
  print_success "nixglhost available (underlying wrapper)"
else
  print_warning "nixglhost not found"
fi

echo ""

# Test 4: Check for GPU (using nvidia-smi from host)
print_header "GPU Detection"

if command -v nvidia-smi &>/dev/null; then
  if nvidia-smi &>/dev/null; then
    GPU_AVAILABLE=true
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
    DRIVER_VERSION=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1)
    print_success "NVIDIA GPU detected: $GPU_NAME"
    print_info "Driver Version: $DRIVER_VERSION"
  else
    print_warning "nvidia-smi found but failed to query GPU"
    print_info "GPU may not be available or driver issue"
  fi
else
  print_warning "nvidia-smi not found - no NVIDIA GPU detected"
  print_info "This is OK for CPU-only development"
fi

echo ""

# Test 5: Compile and run test (if toolkit available)
print_header "CUDA Compilation Test"

if $CUDA_TOOLKIT_OK; then
  TEMP_DIR=$(mktemp -d)
  CUDA_SOURCE="$TEMP_DIR/cuda_test.cu"
  CUDA_BINARY="$TEMP_DIR/cuda_test"

  cat >"$CUDA_SOURCE" <<'CUDA_EOF'
#include <stdio.h>
#include <cuda_runtime.h>

__global__ void test_kernel() {
    printf("GPU kernel executed on thread %d\n", threadIdx.x);
}

int main() {
    int deviceCount = 0;
    cudaError_t err = cudaGetDeviceCount(&deviceCount);

    if (err != cudaSuccess || deviceCount == 0) {
        printf("STATUS:NO_GPU\n");
        return 0;
    }

    cudaDeviceProp prop;
    cudaGetDeviceProperties(&prop, 0);
    printf("STATUS:GPU_FOUND\n");
    printf("GPU_NAME:%s\n", prop.name);
    printf("COMPUTE:%d.%d\n", prop.major, prop.minor);
    printf("MEMORY:%.1f GB\n", prop.totalGlobalMem / (1024.0*1024.0*1024.0));

    test_kernel<<<1, 1>>>();
    cudaDeviceSynchronize();

    err = cudaGetLastError();
    if (err == cudaSuccess) {
        printf("STATUS:KERNEL_OK\n");
    } else {
        printf("STATUS:KERNEL_FAIL\n");
    }

    return 0;
}
CUDA_EOF

  if nvcc -o "$CUDA_BINARY" "$CUDA_SOURCE" 2>/dev/null; then
    print_success "CUDA compilation successful"

    echo ""
    print_header "CUDA Runtime Test"

    if $GPU_AVAILABLE && command -v nixcuda &>/dev/null; then
      OUTPUT=$(nixcuda "$CUDA_BINARY" 2>/dev/null || echo "STATUS:RUNTIME_FAIL")

      if echo "$OUTPUT" | grep -q "STATUS:GPU_FOUND"; then
        CUDA_RUNTIME_OK=true
        GPU_NAME=$(echo "$OUTPUT" | grep "GPU_NAME:" | cut -d: -f2)
        COMPUTE=$(echo "$OUTPUT" | grep "COMPUTE:" | cut -d: -f2)
        MEMORY=$(echo "$OUTPUT" | grep "MEMORY:" | cut -d: -f2)

        print_success "GPU accessible via nixcuda"
        print_info "GPU: $GPU_NAME"
        print_info "Compute Capability: $COMPUTE"
        print_info "Memory: $MEMORY"

        if echo "$OUTPUT" | grep -q "STATUS:KERNEL_OK"; then
          print_success "CUDA kernel execution successful"
        else
          print_error "CUDA kernel execution failed"
        fi
      elif echo "$OUTPUT" | grep -q "STATUS:NO_GPU"; then
        print_warning "No GPU detected at runtime"
      else
        print_error "Runtime test failed"
        print_info "Output: $OUTPUT"
      fi
    elif ! $GPU_AVAILABLE; then
      print_warning "Skipping runtime test - no GPU available"
      print_info "CUDA compilation works, runtime requires GPU"
    else
      print_warning "Skipping runtime test - nixcuda not available"
    fi
  else
    print_error "CUDA compilation failed"
  fi

  rm -rf "$TEMP_DIR"
else
  print_warning "Skipping compilation test - nvcc not available"
fi

echo ""

# Summary
print_header "Summary"
echo ""

if $CUDA_TOOLKIT_OK; then
  print_success "CUDA Toolkit: Available (Nix-managed)"
else
  print_error "CUDA Toolkit: Not available"
fi

if $GPU_AVAILABLE; then
  print_success "NVIDIA GPU: Detected"
else
  print_warning "NVIDIA GPU: Not detected (OK for CPU-only dev)"
fi

if $CUDA_RUNTIME_OK; then
  print_success "CUDA Runtime: Working via nixcuda"
elif $GPU_AVAILABLE; then
  print_warning "CUDA Runtime: Not tested or failed"
else
  print_info "CUDA Runtime: Requires GPU for testing"
fi

echo ""

if $CUDA_TOOLKIT_OK; then
  echo -e "${GREEN}CUDA development environment is ready!${NC}"
  echo ""
  echo "Usage:"
  echo "  Compile:  nvcc -o my_app my_app.cu"
  if $GPU_AVAILABLE; then
    echo "  Run:      nixcuda ./my_app"
  else
    echo "  Run:      nixcuda ./my_app  (requires GPU)"
  fi
  echo ""
  echo -e "${BLUE}Try the visual demo:${NC}"
  echo "  ./software/scripts/cuda-demo.sh"
  echo "  (Matrix multiplication - CPU vs CUDA comparison)"
else
  echo -e "${RED}CUDA development environment is not configured.${NC}"
  echo "Make sure you're in the Nix dev shell: nix develop"
fi

echo ""
