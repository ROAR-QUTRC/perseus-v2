/*
 * Stage 1 decision-gate test (see docs/source/systems/software/
 * isaac-ros-nitros-source-build.md): does the published gxf_aarch64_cuda_13_0
 * (SBSA/server-ARM) libgxf_core.so actually initialize on this Tegra Orin's
 * driver/unified-memory stack, or does it fail at load/init time?
 *
 * Not NVIDIA source — our own minimal test, linked against their built
 * libgxf_core.so via the public C API (gxf/core/gxf.h).
 */
#include <stdio.h>
#include "gxf/core/gxf.h"

int main(void) {
  gxf_context_t context;
  gxf_result_t result = GxfContextCreate(&context);
  if (result != GXF_SUCCESS) {
    printf("GxfContextCreate FAILED: result=%d\n", result);
    return 1;
  }
  printf("GxfContextCreate OK: context=%p\n", (void *)context);

  result = GxfContextDestroy(context);
  if (result != GXF_SUCCESS) {
    printf("GxfContextDestroy FAILED: result=%d\n", result);
    return 1;
  }
  printf("GxfContextDestroy OK\n");
  return 0;
}
