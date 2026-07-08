# NITROS/GXF from-source build experiment (Orin + JetPack 7)

**Status: experimental, staged, not yet attempted on hardware.** This is a
separate track from the working `software/docker/isaac-ros/` scaffold
(NVIDIA's prebuilt image, JetPack 6.x only) — see
[isaac-ros.md](isaac-ros.md). It exists because NVIDIA has not shipped Isaac
ROS binaries for Jetson Orin under JetPack 7 yet, and this robot is now on
JetPack 7 (confirmed 2026-07-08: L4T R39.2.0, CUDA 13.2, Ubuntu 24.04). Rather
than wait, this track attempts to build NITROS (`isaac_ros_nitros`) and its
GXF dependency from source, targeting Orin/JetPack7/CUDA13 directly.

Working files live in `software/docker/isaac-ros/nitros-source/` — see that
directory's README for the actual commands. This page is the "why" and the
staged plan; the README is the "how."

## Why this might work at all

NVIDIA's official Isaac ROS 4.x line (JetPack 7 / Ubuntu 24.04 / ROS 2 Jazzy)
is Jetson Thor–only — confirmed repeatedly by NVIDIA staff:

> "Isaac ROS 4.0 does not currently support Orin platforms."
> — NVIDIA staff, [isaac_ros_nitros#64](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros/issues/64), 2025-11-27

But two things suggest Orin+JetPack7 isn't architecturally excluded, just not
shipped yet:

1. Isaac ROS's own CMake config (`isaac_ros_common-extras.cmake`) explicitly
   compiles fresh aarch64 code for **`CMAKE_CUDA_ARCHITECTURES 87;110;120`** —
   SM 8.7 is Orin's Ampere compute capability, alongside Thor/Spark's
   Blackwell (110/120). This isn't a Thor-only build target by design.
2. A community report from a user running **JetPack 7.2 on an AGX Orin**
   (L4T 39.2, TensorRT 10.16.2.10, CUDA 13.0.85 — essentially this robot's
   stack) found the underlying CUDA/TensorRT/iGPU path functional; the only
   confirmed gap was TensorRT reporting zero DLA cores, which NVIDIA staff
   attributed to DLA support specifically, not the GPU compute path. ([forum
   thread](https://forums.developer.nvidia.com/t/jp-7-2-w-isaac-ros-and-dlas/374021))
3. NVIDIA staff, June 2026: *"We do plan to support JP 7.x on Orin, and this
   is an area the team is actively working on... not ready to share a public
   timeline yet."* ([forum
   thread](https://forums.developer.nvidia.com/t/eta-for-isaac-ros-release-for-jp-7-2-on-orin-devices/372134))

So the hardware/driver layer plausibly works; what's missing is NVIDIA
publishing the matching GXF binaries and doing the integration work.

## Why this might not work (the actual blocker)

`isaac_ros_nitros`'s own C++ (`NitrosNode`, type adapters,
`isaac_ros_managed_nitros`, etc.) is source-available and — once its
dependency resolves — should compile normally. The real blocker is its GXF
(Graph Execution Framework) dependency:

- The build system expects a `gxf_jetpack70/` binary directory for real
  embedded Jetson hardware on JetPack 7. **It does not exist in the public
  repo.** Only `gxf_aarch64_cuda_13_0` and `gxf_x86_64_cuda_13_0` (SBSA —
  server-ARM targets, e.g. DGX Spark / Thor) are published under
  `isaac_ros_nitros/isaac_ros_gxf/gxf/core/lib/`.
- The nominally "buildable GXF source" repo,
  [NVIDIA-ISAAC-ROS/gxf](https://github.com/NVIDIA-ISAAC-ROS/gxf), is
  **stale**: every release tag from `v3.2.0` through `v4.4.0` points at the
  *same* commit (dated 2025-01-10, "Isaac ROS 3.2.0"). That commit's
  `build_gxf_release_content.yaml` only defines `x86_cuda_12_6` and
  `jetpack61` build targets — nothing for JetPack7/CUDA13. The release tags
  are cosmetic; the actual buildable source hasn't moved past the JetPack
  6.1 era.
- `isaac_ros_common`'s `CMAKE_DEVICE` logic doesn't resolve to a Jetson-aware
  value in the generic path either — it deprecated the aarch64→"arm64" branch
  in favor of defaulting to `sbsa`. (The newer `isaac-ros-cli` tool's
  `platform.py` *does* distinguish real Jetson — `ARM64_JETPACK`, detected via
  `/etc/nv_tegra_release` — from DGX Spark's `ARM64_FASTOS`, but that
  detection doesn't currently have a matching binary to select on JetPack7.)

Net: nothing prevents *compiling* NITROS/GXF for SM 8.7 in principle, but the
actual published artifacts to link against for embedded Jetson on
JetPack7/CUDA13 don't exist yet, and the "source you could build them from
yourself" hasn't been updated to target that configuration either.

## Licensing caveats

- `isaac_ros_nitros` carries NVIDIA's source-available **"Isaac ROS Software
  License"**: permits install/use/modify, but restricts use to "systems with
  NVIDIA GPUs" and explicitly forbids relicensing under an open-source
  license (§4f).
- The `gxf` repo's actual core source (`com_nvidia_gxf/`) carries a
  **stricter, more classically proprietary** NVIDIA header: *"Any use,
  reproduction, disclosure or distribution of this software... without an
  express license agreement from NVIDIA CORPORATION is strictly
  prohibited."* This may just be stale boilerplate carried over from GXF's
  internal codebase, but it's a real ambiguity sitting inside a repo NVIDIA
  itself calls "buildable source."
- Practical stance: treat everything under `nitros-source/` as **local
  experimentation only** — nothing from these repos is committed to
  perseus-lite (see `.gitignore`), and nothing built here should be
  redistributed.

## Staged plan

The GXF binary gap is the real unknown, so the plan front-loads the cheapest
experiment that tests it directly, with an explicit decision gate before
committing to the much larger effort of resurrecting GXF's stale build
recipe.

| Stage | Goal | Gate |
| --- | --- | --- |
| 0 | Stand up a working build toolchain on this host (venv/baremetal `isaac-ros-cli`, since Docker isn't installed here) | `colcon build` configures against `isaac_ros_common` cmake macros |
| 1 | Force `CMAKE_DEVICE=sbsa`, link against the *existing* `gxf_aarch64_cuda_13_0` binaries, and run a minimal GXF smoke test on this Tegra hardware | **Decision gate.** Clean failure (ABI/arch mismatch) → binary-reuse path closed, go to Stage 2 or stop. Success → skip Stage 2 entirely |
| 2 | *(only if Stage 1 fails)* Attempt to extend the stale `gxf` repo's build recipe with a `jetpack70`/CUDA13 target | Time-boxed; stop if the `com_nvidia_gxf` licensing ambiguity turns out not to be stale boilerplate, or if progress stalls — fall back to waiting for NVIDIA's stated (undated) JP7-on-Orin work |
| 3 | Once a working `libgxf_core.so` exists (via 1 or 2), build `isaac_ros_nitros` core only (no perception packages) and run a minimal NITROS type-adapter round trip in one process | Round trip actually runs and prints expected content |
| 4 | *(out of scope for now)* `isaac_ros_image_proc` / `isaac_ros_apriltag` — introduces a second independent lock point, `isaac_ros_vpi_utils` (VPI), whose JetPack7 status is unchecked | Not started until Stage 3 succeeds |

Results and exact error output for each stage should be recorded in this
document as they happen, rather than in commit messages or PR descriptions —
this page is the running log.

### Stage 0 — result: succeeded

Cloned `isaac_ros_common`, `isaac_ros_nitros`, `gxf` into `nitros-source/`
(gitignored) without issue.

Standing up a build toolchain hit a bigger snag than expected: **every
documented path requires `sudo`/system-level installation.** After checking
with whoever owns this hardware, went with installing Docker +
`nvidia-container-runtime` directly on the host (matches what the existing
`software/docker/isaac-ros/` scaffold already assumes):

- `sudo apt-get install docker.io nvidia-container-toolkit` — both available
  straight from existing apt sources (`nvidia-container-toolkit` ships on
  NVIDIA's Jetson repo for `r39.2`/JetPack 7 already).
- `sudo nvidia-ctk runtime configure --runtime=docker --set-as-default`,
  `sudo systemctl enable --now docker` — verified with
  `docker run --gpus all nvcr.io/nvidia/l4t-base:r36.2.0 nvidia-smi`, which
  correctly reported the host's driver/GPU through the container (basic
  driver-passthrough sanity check, not a full CUDA-runtime compatibility
  test — see Stage 1).
- Did **not** add the invoking user to the `docker` group (that grants
  persistent root-equivalent host access, beyond what installing Docker
  itself implies) — verification commands use `sudo docker`.
- The newer `isaac-ros-cli` tool turned out **not pip-installable** despite
  its repo being Apache-2.0 — it ships as a `.deb` built via its own
  `Makefile`, which carries the same strict proprietary NVIDIA header
  flagged below for `gxf`'s core ("distribution... without an express
  license agreement... is strictly prohibited") sitting inside an
  otherwise-Apache-2.0 repo. Not pursued — the classic `run_dev.sh`/plain
  colcon route was used instead.
- Also installed: `cuda-toolkit-13-2` (nvcc; only the L4T CUDA *runtime* libs
  were preinstalled, not the compiler) and `vpi4-dev` (NVIDIA's Vision
  Programming Interface — turned out to be an `isaac_ros_common` build
  dependency, not just an `isaac_ros_apriltag` one as originally assumed;
  **VPI 4 is already published for this exact JetPack7/r39.2 repo**, a
  positive sign for that dependency specifically).
- A `pixi` feature (`isaac-nitros`, linux-aarch64 only) provides
  colcon/cmake/compilers in a self-contained Pixi prefix for the actual
  package builds, layered on top of the `default` feature's ROS 2 Jazzy/
  ament tooling (same pattern as `simulation`/`machine-learning`).

**Result: `isaac_ros_common` (the package) builds and installs cleanly** —
`libisaac_ros_common.so` compiled and installed via
`colcon build --packages-select isaac_ros_common` with
`CUDAToolkit_ROOT=/usr/local/cuda`, `CUDACXX=/usr/local/cuda/bin/nvcc`.
Notably, `CMAKE_DEVICE` **auto-detected as `sbsa` on this real Jetson**
without forcing it — confirming the research's prediction that the
build system's generic aarch64 path resolves to the SBSA/server-ARM branch
here, not a dedicated Jetson one.

One more data point worth flagging: the configure log reported
**`CUDA architectures: 75`** (Turing) for this build, not `87` (Orin/Ampere)
as `isaac_ros_common-extras.cmake`'s `CMAKE_CUDA_ARCHITECTURES "87;110;120"`
(found during earlier research) would suggest. Not yet root-caused — could be
a different code path taken under `CMAKE_DEVICE=sbsa` specifically, or a
default applied before that list is set. Compiling for SM 7.5 instead of 8.7
would still likely run on Orin via PTX JIT (if PTX is embedded, not just
cubin) but wouldn't use Orin-native code generation — worth revisiting if
Stage 3 is reached.

### Stage 1 — result: blocked on a build-tooling bug, not (yet) the
architecture question

Attempted `colcon build --packages-select isaac_ros_gxf` — the package that
actually links against the precompiled GXF `.so` binaries. Confirmed live
(matching the research exactly): only `gxf_aarch64_cuda_13_0` and
`gxf_x86_64_cuda_13_0` exist under `isaac_ros_gxf/gxf/core/lib/` — no
`gxf_jetpack70/`.

Never got far enough to test whether those binaries actually load on this
Tegra hardware — hit an earlier, unrelated **CMake generate-time error**:

```
CMake Error at CMakeLists.txt:71 (set_property):
  Error evaluating generator expression:
    $<INSTALL_PREFIX>
  INSTALL_PREFIX is a marker for install(EXPORT) only.  It should never be evaluated.
```

`isaac_ros_gxf/CMakeLists.txt` defines `Core`/`Logger`/`Std`/`Multimedia`/
`Serialization`/`Cuda` as `INTERFACE` libraries with
`INTERFACE_LINK_LIBRARIES`/`INTERFACE_INCLUDE_DIRECTORIES` set to
`$<INSTALL_PREFIX>/...` paths (lines 66–120), then does
`install(TARGETS ... EXPORT export_${PROJECT_NAME})` followed by
`ament_export_targets(...)` and `ament_auto_package(...)`. `$<INSTALL_PREFIX>`
is only valid when evaluated through `install(EXPORT)` machinery — something
in the `ament_export_targets`/`ament_auto_package` combination appears to
evaluate these target properties directly (not through a genuine
`install(EXPORT)` pass), which CMake rejects outright.

**Ruled out a simple CMake-version mismatch**: the package declares
`cmake_minimum_required(VERSION 3.22.1)`, and this failed *identically* on
both CMake 4.3.4 (conda-forge default) and CMake 3.31.8 (pinned down via the
`isaac-nitros` Pixi feature to test the hypothesis). Same error, same lines,
both versions. So this isn't "too-new-CMake" — it's either a bug that's
always been latent in this exact standalone-colcon build path (plausible:
NVIDIA's own CI/testing may never actually exercise the `sbsa` branch of
this file on real Jetson hardware, since Orin+JetPack7 isn't in their
matrix), or something about NVIDIA's actual reference build environment
(their `run_dev.sh` dev container, specific `CMAKE_BUILD_TYPE`/generator/cache
variables) avoids triggering it in a way not yet identified.

**Update: patched and unblocked.** `$<INSTALL_PREFIX>` (12 occurrences) was
replaced with `${CMAKE_INSTALL_PREFIX}` (a plain CMake variable, evaluated at
configure time rather than requiring `install(EXPORT)` context) via `sed` on
the local, gitignored clone — permitted under the Isaac ROS Software
License's modify rights, not redistributed. `isaac_ros_gxf` then built and
installed cleanly.

**Second snag, quickly resolved: the "binaries" were Git LFS pointers, not
real `.so` files.** `libgxf_core.so` etc. were 132-byte ASCII text (LFS
pointer format), because `git-lfs` wasn't installed when the repos were
cloned — CMake's `install(FILES ...)` doesn't validate binary format, so the
build "succeeded" while silently installing placeholder text files. Fixed
with `sudo apt-get install git-lfs` (small, standard package) +
`git lfs install --local && git lfs pull` inside `isaac_ros_nitros/`. After
that, `libgxf_core.so` was a real 2.5 MB aarch64 ELF shared object matching
the LFS pointer's declared size/hash.

### Stage 1 — the actual decision-gate test: SUCCESS

Wrote a minimal C++ smoke test (`nitros-source/gxf_smoke_test.cpp`, our own
code, not NVIDIA's) calling `GxfContextCreate`/`GxfContextDestroy` from
`gxf/core/gxf.h` — the most basic possible exercise of the GXF runtime.

Two more build-recipe snags on the way to actually running it, both worth
recording for whoever continues this:

1. **Link with the system compiler, not Pixi's conda-forge cross-toolchain.**
   Linking with `aarch64-conda-linux-gnu-g++` (the compiler `isaac-nitros`'s
   `cxx-compiler` package provides) failed with `undefined reference` to
   versioned glibc symbols (`dlopen@GLIBC_2.34`, `__isoc23_strtoull@GLIBC_2.38`,
   etc.) — conda-forge's toolchain targets an old, portable glibc baseline by
   design, but NVIDIA's binaries are built natively against Ubuntu 24.04's
   glibc 2.39. Switched to the host's own `/usr/bin/g++` (already present via
   `build-essential`, pulled in as a dependency somewhere along the way) for
   this link step, which resolved it immediately.
2. **`RUNPATH` doesn't propagate transitively.** `libgxf_core.so` itself
   needs `libgxf_logger.so`; setting `-Wl,-rpath` on our executable covers
   *our* direct link but not `libgxf_core.so`'s own `NEEDED` entries (modern
   `DT_RUNPATH`, unlike legacy `DT_RPATH`, only applies to the object that
   carries it, not what that object subsequently loads). Ran with
   `LD_LIBRARY_PATH` set instead.

Result:

```
$ ./gxf_smoke_test
GxfContextCreate OK: context=0xaaaafcddaa00
GxfContextDestroy OK
```

**The published `gxf_aarch64_cuda_13_0` (SBSA/server-ARM) GXF core binary
loads and initializes correctly on this Jetson Orin Nano's Tegra driver/
unified-memory stack under JetPack 7.** This directly answers the Stage 1
decision gate: the "reuse NVIDIA's existing binaries" path is **open**, not
closed. **Stage 2 (resurrecting the stale `gxf` build recipe to produce a
dedicated `gxf_jetpack70` target) can likely be skipped entirely** — there's
no evidence yet that the SBSA build is unsuitable for Orin at the level
tested (context lifecycle only; no CUDA kernel execution, no NITROS message
passing, no perception pipeline).

Caveats on how far this result reaches:
- Only tested `GxfContextCreate`/`GxfContextDestroy` — no GXF graph execution,
  no CUDA interop (`libgxf_cuda.so`), no NITROS layer above this yet.
- Still unresolved: the `CUDA architectures: 75` (Turing) vs `87`
  (Orin/Ampere) discrepancy noted in Stage 0 — irrelevant to *this* binary
  (which is prebuilt, not something we compiled), but relevant once Stage 3
  compiles fresh NITROS C++ code that needs to target Orin's actual SM
  correctly.
- This is one data point on one board. Not a substitute for NVIDIA's own
  validation, and not something to treat as "Orin is officially supported."

### Stage 3 — result: SUCCESS, real NITROS round trip running on Orin/JetPack7

Built `isaac_ros_nitros` (the `NitrosNode`/`NitrosContext`/type-adapter core)
and its full dependency chain — `isaac_ros_common`, `isaac_ros_gxf`, all 17
`gxf_isaac_*` extension packages, `negotiated`/`negotiated_interfaces`
(cloned separately from
[osrf/negotiated](https://github.com/osrf/negotiated), Apache-2.0/Boost,
gitignored same as the other clones — not in RoboStack) — via
`colcon build --packages-up-to isaac_ros_nitros`.

Three more snags on the way, all resolved:

1. **The `$<INSTALL_PREFIX>` bug from Stage 1 is systemic, not a one-off.**
   17 `CMakeLists.txt` files across `isaac_ros_managed_nitros` and nearly
   every `gxf_isaac_*` extension use the identical broken pattern (clearly
   generated from a shared template). Batch-patched all 17 with the same
   `sed` substitution.
2. **`magic_enum` is a real, correctly-declared dependency
   (`isaac_ros_gxf/package.xml` already lists it) that just isn't installed
   anywhere.** Added `magic_enum` to the `isaac-nitros` Pixi feature
   (conda-forge has it, MIT-licensed). Once installed, `isaac_ros_gxf`
   linked fine, but consuming code failed with
   `fatal error: magic_enum.hpp: No such file or directory` — **conda-forge
   packages it under `include/magic_enum/magic_enum.hpp`, but NVIDIA's code
   does a flat `#include "magic_enum.hpp"`, expecting it directly on the
   include path** (their own build presumably vendors a single-file copy
   rather than this subdirectory layout). Worked around with
   `CXXFLAGS="-I$CONDA_PREFIX/include/magic_enum"` rather than patching
   NVIDIA's `#include` lines. Needed a clean `rm -rf build/isaac_ros_nitros
   install/isaac_ros_nitros` afterward — CMake caches
   `CMAKE_CXX_FLAGS` at first configure and won't pick up an env var change
   without a fresh configure.
3. Built cleanly after that (some harmless C++20-designated-initializer
   warnings — the code assumes `-std=c++20`, worth setting explicitly if
   this becomes a permanent build, not investigated further here).

**NVIDIA ships its own minimal NITROS proof-of-life test** —
`isaac_ros_nitros/test/isaac_ros_nitros_test_pol.py` — built automatically
since `BUILD_TESTING` defaults on: two chained `NitrosEmptyForwardNode`
composable nodes in one `component_container_mt`, publish `std_msgs/Empty`
on one end, verify it arrives on the other after passing through NITROS
negotiation and GXF graph execution twice. Running it via the official
`isaac_ros_test`/`launch_testing`/`pytest.mark.rostest` harness would have
needed `pip install torch` — `isaac_ros_test/__init__.py` eagerly imports
`MockModelGenerator`, which imports `torch`, for a model-mocking helper
unrelated to this test. Skipped that weight for a **minimal** round trip:
wrote our own `nitros_roundtrip_launch.py` (replicates the exact
`ComposableNodeContainer` setup) and `nitros_roundtrip_check.py` (plain
`rclpy` publish/subscribe, no `launch_testing`/`isaac_ros_test`) instead —
both in `nitros-source/`, our own code, not NVIDIA's.

One bug in our own launch file along the way: the second node's remappings
used relative topic names (`'mid1/topic_forward_input'`), which resolved
*relative to that node's own `mid1` namespace* — silently doubling to
`/mid1/mid1/topic_forward_input`, which matched nothing. Fixed by making the
remap `from`/`to` strings absolute (leading `/`).

Result, run on this Jetson Orin Nano, JetPack 7, `ROS_DOMAIN_ID=51`:

```
$ ros2 launch nitros_roundtrip_launch.py &
...
[nitros_stage1]: [NitrosNode] Starting negotiation...
[nitros_stage1]: [NitrosPublisher] Use only the compatible publisher: topic_name="/topic_forward_output", data_format="nitros_empty"
[nitros_stage1]: [NitrosNode] Wrote the final top level YAML graph to "/tmp/isaac_ros_nitros/graphs/.../....yaml"
[nitros_stage1]: [NitrosNode] Initializing and running GXF graph
[nitros_stage1]: [NitrosNode] Node was started
[mid1.nitros_stage2]: [NitrosNode] Node was started
...

$ python3 nitros_roundtrip_check.py
[nitros_roundtrip_check]: sent=1 received=1
ROUNDTRIP OK
```

Also ran NVIDIA's own `test_cuda_stream_pool` gtest (built automatically
alongside the round-trip test, exercises `libgxf_cuda.so` directly — real
CUDA stream acquire/release/reuse/overflow, not just context lifecycle):
**14/14 passed.**

**What this establishes:** real NITROS negotiation, GXF graph
construction/execution, and CUDA stream management via the `gxf_aarch64_cuda_13_0`
(SBSA) binaries all work correctly on this Orin Nano under JetPack 7 — well
beyond Stage 1's basic context-lifecycle check. This is still one board, one
minimal (`nitros_empty` — no actual payload/tensor data) message type, and
no camera/perception pipeline (`isaac_ros_apriltag`/`isaac_ros_image_proc`,
gated on `isaac_ros_vpi_utils`, is Stage 4 — not started).

Still unresolved from Stage 0: the `CUDA architectures: 75` vs `87`
discrepancy — now more relevant, since Stage 3 compiled real NITROS C++
(unlike Stage 1's prebuilt GXF binaries). Worth checking whether Orin-native
(SM 8.7) codegen is actually happening before scaling this up.

### Stage 4 — result: SUCCESS, real AprilTag detection matches NVIDIA's exact ground truth

Built `isaac_ros_apriltag` (real perception, not just NITROS plumbing) and
its dependency chain: `isaac_ros_vpi_utils`, `isaac_ros_cvcuda_utils`,
`isaac_ros_image_proc`, `isaac_ros_nitros_camera_info_type`. Two more repos
cloned (same gitignore treatment as before, not committed):
[NVIDIA-ISAAC-ROS/isaac_ros_apriltag](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag)
and
[NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline)
(contains `isaac_ros_image_proc`, `isaac_ros_cvcuda_utils`,
`isaac_ros_vpi_utils`, plus depth/stereo variants we didn't need).

**A new external dependency, resolved via a different channel than
Isaac ROS's own binaries:** `isaac_ros_image_proc` (and `isaac_ros_cvcuda_utils`)
directly requires NVIDIA's **CV-CUDA** library (`cvcuda0-dev`), which isn't in
any apt repo here. Unlike GXF, CV-CUDA is a
[separately maintained, actively released open-source project](https://github.com/CVCUDA/CV-CUDA)
(Apache-2.0) with its own GitHub releases — found a prebuilt `.deb` matching
this exact platform: `cvcuda-{lib,dev}-0.16.0-cuda13-aarch64-linux.deb`.
Downloaded and `sudo dpkg -i`'d both (lib first, then dev).

Four more build-recipe snags, all following patterns already established in
Stages 1/3:

1. **VPI/CV-CUDA headers not found despite being installed** — same root
   cause as `magic_enum` in Stage 3: conda-forge's cross-compiler doesn't
   search `/usr/include` (even though `/usr/include/vpi` and
   `/usr/include/cvcuda` are valid `update-alternatives` symlinks to
   `/opt/nvidia/{vpi4,cvcuda0}/include`). Fixed by pointing `CXXFLAGS`
   directly at `/opt/nvidia/vpi4/include` and `/opt/nvidia/cvcuda0/include`
   — **not** `-I/usr/include` broadly, which was tried first and **broke
   the build worse**: it made the conda cross-compiler pick up the host's
   `glibc` `features.h` ahead of its own bundled one, cascading into
   `bits/timesize.h: No such file or directory` (a header that exists in
   Ubuntu's glibc layout but not where conda's toolchain expects it). Scope
   `-I` additions to the exact directory needed, never a broad system path.
2. **`isaac_ros_cvcuda_utils`'s library needed `-L`/`-rpath` for
   `/opt/nvidia/cvcuda0/lib`** too — added via `LDFLAGS`.
3. **Test executables in `isaac_ros_cvcuda_utils` hit the same
   conda-toolchain-vs-native-glibc linking failure as the Stage 1 GXF smoke
   test** (`undefined reference to dlopen@GLIBC_2.34` etc.) — but this time
   there were many of them across multiple packages, so re-linking each
   individually with the system compiler wasn't practical. Used
   `--cmake-args -DBUILD_TESTING=OFF` instead once the actual libraries were
   confirmed building fine; skips gtest binaries for these packages, not
   full build breakage.
4. **`colcon build --packages-select isaac_ros_apriltag` alone fails** even
   though `isaac_ros_image_proc` is only an `exec_depend`, not a `depend` —
   colcon's `ament_cmake` build task sources every dependency's environment
   hook (`package.sh`) regardless of depend type, so `isaac_ros_image_proc`
   genuinely has to be built first regardless of whether apriltag's own
   compiled code touches it.
5. `isaac_ros_image_proc` itself just took a while to compile (real CUDA/
   CV-CUDA kernel code) — ran it in the background rather than assuming a
   hang.

**Verification — not just "it compiles," but a pixel-exact match against
NVIDIA's own test fixture.** `isaac_ros_apriltag` ships a real ground-truth
test case (`isaac_ros_apriltag/test/test_cases/apriltag0/`: a `1920x1080`
PNG with a physical `tag36h11` id=0 tag, matching `camera_info.json`, and
exact expected corner/center/pose values baked into
`isaac_ros_apriltag_pol_test.py`). Running that official test needs
`isaac_ros_test` (torch, again) — wrote `apriltag_check.py` instead: loads
the same PNG via `cv_bridge`, publishes it + the camera info, subscribes to
`tag_detections`, and checks the detection against NVIDIA's exact expected
values (2px corner/center tolerance, matching their own test).

```
$ ros2 launch apriltag_launch.py &
...
[apriltag]: Using cuAprilTag implementation.
[INFO] [launch_ros.actions.load_composable_nodes]: Loaded node '/apriltag' in container '/apriltag_container'

$ python3 apriltag_check.py
[apriltag_check]: sent=5 received=True
id=0 family=tag36h11 center=(926.0,547.0) expected_center=(926.0, 547.0)
APRILTAG OK
```

Exact pixel match (`926.0, 547.0`) against NVIDIA's own precomputed ground
truth — this is real `cuAprilTag`/VPI-based detection producing correct
results, on this Orin Nano, under JetPack 7, end to end: image →
NITROS-wrapped tensor → VPI/cuAprilTag GPU detection → `AprilTagDetectionArray`.

**What Stage 4 establishes beyond Stage 3:** actual GPU compute correctness
(not just "the process didn't crash") for a real perception algorithm,
across three additional NVIDIA-maintained native libraries (VPI, CV-CUDA,
plus the AprilTag detector itself) all functioning correctly together on
this hardware/JetPack combination.

### Stage 4 follow-up — live camera (Logitech C920): works, with a real bug found in rectify+apriltag

Added `ros-jazzy-usb-cam` to the `isaac-nitros` Pixi feature and a
`pixi run -e isaac-nitros test-apriltags` task
(`run_test_apriltags.sh` → `apriltag_camera_launch.py`) for a live-camera
test, since the ground-truth ap test above only used a static image.

**Two more issues found, one fixed, one worked around (not yet root-caused):**

1. **Resolution/calibration mismatch in NVIDIA's own bundled config.**
   `isaac_ros_apriltag_usb_cam.launch.py` + `usb_cam_params.yaml` doesn't
   set `image_width`/`image_height`, so `usb_cam` defaults to capturing
   640×480 — but the bundled `camera_info.yaml` it loads is calibrated for
   1280×720. `usb_cam` still publishes that mismatched calibration on
   `/camera_info` without erroring, and `RectifyNode` then produces a
   wrongly-scaled `/image_rect`. Fixed by writing our own
   `camera_c920_params.yaml` (1280×720 capture, matching a
   `camera_c920_info.yaml` also at 1280×720) and our own launch file rather
   than editing NVIDIA's installed config (which colcon would overwrite on
   rebuild anyway). **Caveat:** `camera_c920_info.yaml`'s intrinsics are
   NVIDIA's own placeholder test values, not a real calibration for this
   physical C920 — fine for demoing detection, not for trusting the
   reported pose. A real calibration needs
   `ros2 run camera_calibration cameracalibrator` with a checkerboard.

2. **`rectify` → `apriltag` on live camera frames reproducibly crashes —
   not yet root-caused.**
   ```
   terminate called after throwing an instance of 'nvcv::Exception'
     what():  NVCV_ERROR_INVALID_OPERATION: The tensor handle is null.
   ```
   Isolated carefully (each combination tested independently, with a clean
   process state — an earlier false lead came from a zombie `ros2 launch`
   process left over from a prior crash still holding the `apriltag_container`
   node name, which produced a *different*, misleading symptom: the next
   launch attempt hung indefinitely with zero node-loading progress. Always
   force-kill (`pkill -9 -f component_container_mt`) and verify with `ps`
   before re-testing after a crash.):
   - `apriltag` alone (static test image, no rectify): works
     (Stage 4 ground-truth test, and again standalone here).
   - `rectify` + `apriltag` loaded together, **no live frames flowing**
     (no publisher feeding rectify): loads fine, no crash.
   - `usb_cam` + `rectify` + `apriltag`, **live frames actively flowing**:
     crashes consistently, within ~1 second of `apriltag` finishing its
     `load_node` call — i.e. specifically when `apriltag` starts actually
     consuming `rectify`'s live NITROS-wrapped image output, not during
     either node's own initialization alone.
   - Node load order doesn't matter (tried apriltag-before-usb_cam too;
     same crash once real frames reach the rectify→apriltag handoff).

   This points at the interaction between `RectifyNode`'s NITROS/CV-CUDA
   image output and `AprilTagNode`'s consumption of it under real
   throughput — plausibly related to the still-unresolved `CUDA
   architectures: 75` vs `87` question from Stage 0, or a CV-CUDA
   version/ABI mismatch specific to that data path, but not confirmed.
   **Worked around, not fixed:** `apriltag_camera_launch.py` skips
   `RectifyNode` entirely and feeds `usb_cam`'s raw image straight to
   `AprilTagNode`. Verified stable (`/tag_detections` publishing steadily
   at 10 Hz, matching the camera's framerate, no crash) — but the C920's
   lens distortion goes uncorrected, so corner/pose accuracy is worse than
   the rectified path would give. Revisit this if accurate pose (not just
   "is a tag visible") matters later.

**Result:** live AprilTag detection works —
`pixi run -e isaac-nitros test-apriltags` launches `usb_cam` (1280×720,
10 Hz) → `apriltag` directly, publishing `/tag_detections` and per-tag TF
frames (`tag36h11:<id>`) at a steady 10 Hz. RViz visualization from a
second machine: see `nitros-source/README.md` "Live camera test."
