{ pkgs, ... }:
{
  allowUnfreePredicate =
    pkg:
    builtins.elem (pkgs.lib.getName pkg) [
      # The docs need drawio for graphs
      "drawio"
      # CUDA packages
      "cuda_cudart"
      "cuda_nvcc"
      "cuda_cccl"
      "libcublas"
      "libcufft"
      "libcurand"
      "libcusolver"
      "libcusparse"
      "libnpp"
      "cuda_nvrtc"
      "cuda_nvml_dev"
      "cuda_profiler_api"
      "cudatoolkit"
      "libnvjitlink"
      "cuda_cuobjdump"
      "cuda_gdb"
      "cuda_cuxxfilt"
      "cuda_nvdisasm"
      "cuda_nvprune"
      "cuda_sanitizer_api"
      "cuda_nvtx"
      # ONNX Runtime CUDA dependencies
      "cudnn"
      "cudnn-frontend"
    ];
}
