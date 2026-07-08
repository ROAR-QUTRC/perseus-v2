# NITROS/GXF from-source build experiment (Orin + JetPack 7)

**Status: Stages 0–4 all succeeded.** Real AprilTag detection
(`isaac_ros_apriltag`, VPI/cuAprilTag) running on this Jetson Orin Nano
under JetPack 7 using entirely self-built binaries, verified to a **pixel-exact
match** against NVIDIA's own ground-truth test fixture — see Stage 4 below.
This is a separate track from the working `software/docker/isaac-ros/`
scaffold (which targets JetPack 6.x via NVIDIA's prebuilt image) — it exists
because NVIDIA hasn't shipped Isaac ROS binaries for Orin+JetPack7 yet, and
we're building our own from source as a stopgap. Not yet exercised: a live
camera (Stage 4 used a static test image) and the rectify half of the real
camera→rectify→apriltag pipeline.

Full background, findings, and the staged plan: see
`docs/source/systems/software/isaac-ros-nitros-source-build.md`. Read that
before running anything here — it explains *why* this is staged the way it
is and what the licensing caveats are.

**Nothing cloned here is committed to perseus-lite.** `isaac_ros_common/`,
`isaac_ros_nitros/`, `gxf/`, and any `build/`/`install/`/`log/` output are
git-ignored (see repo-root `.gitignore`) — they're NVIDIA source-available /
proprietary-licensed code, not ours to redistribute. Only this README and any
scripts we write ourselves live in version control.

## Stage 0 — clone + environment stand-up

```console
cd software/docker/isaac-ros/nitros-source
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros
git clone https://github.com/NVIDIA-ISAAC-ROS/gxf
```

This host has no `docker` installed (checked 2026-07-08), so the classic
`isaac_ros_common/scripts/run_dev.sh` container workflow isn't available
without first standing up Docker + nvidia-container-runtime.

**Correction (found while attempting this live):** `isaac-ros-cli` is
**not** pip-installable — despite the outer repo being Apache-2.0, it ships
as a `.deb` (`sudo apt-get install isaac-ros-cli`, not in this host's apt
sources yet) built via its own `Makefile`, and even `isaac-ros init` itself
requires `sudo`. That `Makefile` also carries the same strict proprietary
NVIDIA header we already flagged for `gxf`'s core source in the doc page
("distribution... without an express license agreement... is strictly
prohibited") — another repo nominally open but with a stricter notice buried
inside.

So every documented path to a working Isaac ROS build environment — the CLI
tool, or the classic `run_dev.sh` container flow — crosses into installing
system packages / Docker with `sudo` at some point. Pixi deliberately can't
and shouldn't paper over that (see the pixi.toml `isaac-nitros` feature
comment). **Don't run any `sudo apt`/`sudo make install` here without
checking with whoever owns this box first** — this is real system state on
real robot hardware, not something scoped to the repo. See the doc page's
"Stage 0" section for the options and what was decided.

Once a toolchain is available (system CLI, Docker, or a manual bypass), the
`isaac-nitros` Pixi env provides the plain colcon/cmake/compiler toolchain
for the actual package builds: `pixi shell -e isaac-nitros`. Success
criterion for this stage: `colcon build` at least *configures* against
`isaac_ros_common`'s CMake macros — it is expected to fail at the GXF
resolution step (that's Stage 1).

## Stage 1 — does the published SBSA GXF binary load on Tegra?

**Result: yes.** See `isaac-ros-nitros-source-build.md` for the full story.
Summary of what actually worked, in order:

```console
# git-lfs is required — isaac_ros_nitros's .so files are LFS objects; without
# this they clone as ~132-byte text pointers that CMake happily "installs"
# without complaining, only failing much later at link time.
sudo apt-get install git-lfs
cd isaac_ros_nitros && git lfs install --local && git lfs pull && cd ..

# CMAKE_DEVICE auto-detects to sbsa on real Jetson hardware already (no
# forcing needed — only gxf_aarch64_cuda_13_0/gxf_x86_64_cuda_13_0 exist,
# no gxf_jetpack70/ yet).
#
# isaac_ros_gxf/CMakeLists.txt needed a local patch first: it uses
# $<INSTALL_PREFIX> (a generator expression only valid inside
# install(EXPORT)) directly in INTERFACE_LINK_LIBRARIES/
# INTERFACE_INCLUDE_DIRECTORIES, which newer CMake rejects outright
# ("should never be evaluated") — confirmed NOT a CMake-version issue
# (fails identically on 3.31.8 and 4.3.4). Fixed locally with:
sed -i 's/\$<INSTALL_PREFIX>/${CMAKE_INSTALL_PREFIX}/g' \
  isaac_ros_nitros/isaac_ros_gxf/CMakeLists.txt

mkdir -p ws/src
ln -sfn ../../isaac_ros_common ws/src/isaac_ros_common
ln -sfn ../../isaac_ros_nitros ws/src/isaac_ros_nitros
cd ws
pixi run -e isaac-nitros bash -c '
  export CUDAToolkit_ROOT=/usr/local/cuda
  export CUDACXX=/usr/local/cuda/bin/nvcc
  export PATH=/usr/local/cuda/bin:$PATH
  colcon build --packages-select isaac_ros_common isaac_ros_gxf
'
cd ..
```

Then the actual smoke test (`gxf_smoke_test.cpp` in this directory —
`GxfContextCreate`/`GxfContextDestroy`, our own minimal code, not NVIDIA's).
**Link with the host's own `g++`, not Pixi's `isaac-nitros` conda-forge
cross-compiler** — the conda toolchain targets an old portable glibc
baseline and can't resolve versioned symbols
(`dlopen@GLIBC_2.34`, `__isoc23_strtoull@GLIBC_2.38`, ...) that NVIDIA's
natively-built (Ubuntu 24.04, glibc 2.39) binaries require:

```console
GXF_INSTALL=ws/install/isaac_ros_gxf
g++ gxf_smoke_test.cpp -o gxf_smoke_test \
  -I"$GXF_INSTALL/share/isaac_ros_gxf/gxf/include" \
  -L"$GXF_INSTALL/lib" -lgxf_core \
  -L"$GXF_INSTALL/share/isaac_ros_gxf/gxf/lib/logger" -lgxf_logger

# RUNPATH doesn't propagate transitively (libgxf_core.so's own NEEDED
# entries, like libgxf_logger.so, aren't resolved by our binary's rpath) —
# use LD_LIBRARY_PATH instead of fighting rpath for a quick smoke test:
LD_LIBRARY_PATH="$PWD/$GXF_INSTALL/lib:$PWD/$GXF_INSTALL/share/isaac_ros_gxf/gxf/lib/logger" \
  ./gxf_smoke_test
```

Expected output: `GxfContextCreate OK: context=0x...` /
`GxfContextDestroy OK`. This is the Stage 1 decision gate — success here
means Stage 2 (resurrecting the stale `gxf` build recipe) is likely
unnecessary; proceed to Stage 3 (build `isaac_ros_nitros` core) instead.

## Stage 2 — skipped

Not attempted. Stage 1 succeeded, so resurrecting the stale `gxf` build
recipe wasn't necessary.

## Stage 3 — build `isaac_ros_nitros` core + minimal round trip

**Result: yes, real NITROS round trip works.** See
`isaac-ros-nitros-source-build.md` for the full story (three more snags, all
resolved: the `$<INSTALL_PREFIX>` bug turned out to be in 17 files, not one;
`magic_enum` needed adding to the Pixi env plus a `CXXFLAGS` include-path
workaround for a conda-forge-vs-NVIDIA packaging-layout mismatch; and a
remapping bug in our own launch file). Commands:

```console
# Two more repos needed beyond Stage 0/1's clones:
# - isaac_ros_gxf_extensions is already inside isaac_ros_nitros (no new clone)
# - negotiated is a separate, fully open-source (Apache-2.0/Boost) repo:
git clone --depth 1 https://github.com/osrf/negotiated

# (isaac_ros_gxf_extensions needs no separate symlink -- colcon already
# recurses into isaac_ros_nitros/, which contains it)
ln -sfn ../../negotiated/negotiated ws/src/negotiated
ln -sfn ../../negotiated/negotiated_interfaces ws/src/negotiated_interfaces

# Same $<INSTALL_PREFIX> bug, 17 files this time (isaac_ros_managed_nitros +
# nearly every gxf_isaac_* extension — clearly from a shared template):
grep -rl '\$<INSTALL_PREFIX>' isaac_ros_nitros/ --include="CMakeLists.txt" \
  | xargs sed -i 's/\$<INSTALL_PREFIX>/${CMAKE_INSTALL_PREFIX}/g'

cd ws
pixi run -e isaac-nitros bash -c '
  export CUDAToolkit_ROOT=/usr/local/cuda
  export CUDACXX=/usr/local/cuda/bin/nvcc
  export PATH=/usr/local/cuda/bin:$PATH
  # magic_enum: added to the isaac-nitros Pixi feature (conda-forge, MIT).
  # It packages the header under include/magic_enum/magic_enum.hpp; NVIDIA
  # code does a flat #include "magic_enum.hpp" -- point CXXFLAGS at the
  # subdirectory rather than patching their #include lines:
  export CXXFLAGS="-I$CONDA_PREFIX/include/magic_enum"
  colcon build --packages-up-to isaac_ros_nitros
'
cd ..
```

`BUILD_TESTING` defaults on, so this also builds NVIDIA's own minimal NITROS
proof-of-life test infra: `libnitros_empty_forward_node.so` (a composable
node plugin) and `test_cuda_stream_pool` (a standalone gtest — run it
directly, it needs no launch/ROS graph):

```console
GXF_LIB_DIRS=$(find ws/install -type d -path "*/gxf/lib/*" ! -name test | tr '\n' ':')
INSTALL_LIB_DIRS=$(find ws/install -maxdepth 2 -type d -name lib | tr '\n' ':')
pixi run -e isaac-nitros bash -c "
  source ws/install/setup.bash
  export LD_LIBRARY_PATH=\"$GXF_LIB_DIRS$INSTALL_LIB_DIRS\$LD_LIBRARY_PATH\"
  ./ws/build/isaac_ros_nitros/test_cuda_stream_pool
"
# Expect: [ PASSED ] 14 tests. -- real CUDA stream acquire/release/reuse via
# libgxf_cuda.so, not just context lifecycle.
```

For the actual round trip: NVIDIA's own test
(`isaac_ros_nitros/test/isaac_ros_nitros_test_pol.py`) needs the
`isaac_ros_test`/`launch_testing`/`pytest.mark.rostest` harness, which pulls
in `torch` transitively (`isaac_ros_test/__init__.py` eagerly imports a
model-mocking helper unrelated to this test) — too heavy for a minimal
check. Used `nitros_roundtrip_launch.py` + `nitros_roundtrip_check.py` in
this directory instead (our own code, replicating the same
`ComposableNodeContainer` setup minus the torch dependency):

```console
pixi run -e isaac-nitros bash -c "
  source ws/install/setup.bash
  export LD_LIBRARY_PATH=\"$GXF_LIB_DIRS$INSTALL_LIB_DIRS\$LD_LIBRARY_PATH\"
  ros2 launch nitros_roundtrip_launch.py &
"
sleep 4   # let negotiation + GXF graph load finish
pixi run -e isaac-nitros bash -c '
  source ws/install/setup.bash
  python3 nitros_roundtrip_check.py
'
# Expect: sent=1 received=1 / ROUNDTRIP OK
pkill -f component_container_mt   # clean up when done
```

If you see `sent=N received=0` with the container log showing both nodes
"Node was started" — check `ros2 topic list` for whether your remap
`from`/`to` strings are absolute (leading `/`). A relative remap resolves
*inside the node's own namespace* — `('mid1/x', 'y')` on a node namespaced
`mid1` silently becomes `/mid1/mid1/x`, matching nothing.

## Stage 4 — real perception: `isaac_ros_apriltag` + `isaac_ros_image_proc`

**Result: yes — pixel-exact match against NVIDIA's own ground truth.** See
`isaac-ros-nitros-source-build.md` for the full story. Two more repos, and a
dependency resolved through a different channel than Isaac ROS's own
binaries (NVIDIA's separately-released, actively-maintained
[CV-CUDA](https://github.com/CVCUDA/CV-CUDA), Apache-2.0):

```console
git clone --depth 1 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag
# This repo is large enough that a plain `git clone` can time out fetching
# LFS content — skip the smudge, clone fast, then pull LFS deliberately:
GIT_LFS_SKIP_SMUDGE=1 git clone --depth 1 \
  https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline
cd isaac_ros_image_pipeline && git lfs install --local && git lfs pull && cd ..

ln -sfn ../../isaac_ros_apriltag ws/src/isaac_ros_apriltag
ln -sfn ../../isaac_ros_image_pipeline ws/src/isaac_ros_image_pipeline

# CV-CUDA (cvcuda0-dev) isn't in any apt repo, but CV-CUDA publishes its own
# GitHub release .debs -- find the one matching this platform (CUDA 13,
# aarch64) and install lib before dev:
curl -fSLO https://github.com/CVCUDA/CV-CUDA/releases/download/v0.16.0/cvcuda-lib-0.16.0-cuda13-aarch64-linux.deb
curl -fSLO https://github.com/CVCUDA/CV-CUDA/releases/download/v0.16.0/cvcuda-dev-0.16.0-cuda13-aarch64-linux.deb
sudo dpkg -i cvcuda-lib-0.16.0-cuda13-aarch64-linux.deb
sudo dpkg -i cvcuda-dev-0.16.0-cuda13-aarch64-linux.deb

cd ws
pixi run -e isaac-nitros bash -c '
  export CUDAToolkit_ROOT=/usr/local/cuda
  export CUDACXX=/usr/local/cuda/bin/nvcc
  export PATH=/usr/local/cuda/bin:$PATH
  # VPI and CV-CUDA headers: same conda-sysroot issue as magic_enum in Stage 3
  # -- point CXXFLAGS at the exact include dirs. Do NOT add a bare
  # -I/usr/include "to be safe": it shadows condas own glibc headers with the
  # hosts and breaks the build worse (bits/timesize.h: No such file, in gtest).
  export CXXFLAGS="-I$CONDA_PREFIX/include/magic_enum -I/opt/nvidia/vpi4/include -I/opt/nvidia/cvcuda0/include"
  export LDFLAGS="-L/opt/nvidia/cvcuda0/lib -Wl,-rpath,/opt/nvidia/cvcuda0/lib"
  # BUILD_TESTING=OFF: several gtest binaries in these packages hit the same
  # conda-toolchain-vs-native-glibc link failure as the Stage 1 smoke test,
  # too many to individually re-link with the system compiler. The actual
  # libraries build and link fine either way.
  colcon build --packages-select isaac_ros_cvcuda_utils --cmake-args -DBUILD_TESTING=OFF
  colcon build --packages-select isaac_ros_image_proc --cmake-args -DBUILD_TESTING=OFF
  colcon build --packages-select isaac_ros_apriltag --cmake-args -DBUILD_TESTING=OFF
'
cd ..
```

`isaac_ros_image_proc` takes a few minutes (real CUDA/CV-CUDA kernel
compilation) — run it in the background rather than assuming a hang.
`colcon build --packages-select isaac_ros_apriltag` alone fails even though
`isaac_ros_image_proc` is only its `exec_depend`: colcon's `ament_cmake`
build task sources every dependency's environment hook regardless of
depend type, so `image_proc` has to actually be built first.

Verification uses NVIDIA's own ground-truth fixture
(`isaac_ros_apriltag/isaac_ros_apriltag/test/test_cases/apriltag0/`) rather
than a synthetic message, via `apriltag_launch.py` +
`apriltag_check.py` in this directory (our own code — loads the same PNG
via `cv_bridge`, bypassing `isaac_ros_test`'s torch dependency same as
Stage 3):

```console
GXF_LIB_DIRS=$(find ws/install -type d -path "*/gxf/lib/*" ! -name test | tr '\n' ':')
INSTALL_LIB_DIRS=$(find ws/install -maxdepth 2 -type d -name lib | tr '\n' ':')
EXTRA_LIB_DIRS="/opt/nvidia/vpi4/lib/aarch64-linux-gnu:/opt/nvidia/cvcuda0/lib:"

pixi run -e isaac-nitros bash -c "
  source ws/install/setup.bash
  export LD_LIBRARY_PATH=\"$GXF_LIB_DIRS$INSTALL_LIB_DIRS$EXTRA_LIB_DIRS\$LD_LIBRARY_PATH\"
  ros2 launch apriltag_launch.py &
"
sleep 5
pixi run -e isaac-nitros bash -c '
  source ws/install/setup.bash
  python3 apriltag_check.py
'
# Expect: id=0 family=tag36h11 center=(926.0,547.0) expected_center=(926.0, 547.0)
#         APRILTAG OK
pkill -f component_container_mt   # clean up when done
```

Not yet exercised: a live camera (this test publishes a static PNG, not
`usb_cam`/Argus) and `isaac_ros_image_proc::RectifyNode` (the ground truth
fixture bypasses rectify) — the real deployed graph is
camera → rectify → apriltag, and only the apriltag stage has been validated
in isolation so far.
