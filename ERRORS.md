# Error Pattern Log

### Non-portable `-march=native` baked into a binary-cached Nix package — 2026-06-19

- **Severity:** High
- **Category:** Configuration
- **File(s):** `software/ros_ws/third-party-packages/vikit-common/default.nix` (upstream `vikit_common/CMakeLists.txt`)
- **Pattern:** Vendored third-party CMake projects that hardcode `-march=native` (or other host-specific `-mtune=native`/auto-ISA flags) in `CMAKE_CXX_FLAGS`. When the artifact is built on CI and pulled from a binary cache onto different hardware, the baked-in ISA can trigger `SIGILL` (illegal instruction) at runtime. Also makes the build non-reproducible.
- **Root cause:** Upstream `vikit_common` targets a single known machine and optimizes for it; that assumption is invalid for a shared/cached Nix derivation deployed to heterogeneous robot CPUs.
- **Fix applied:** Added `--replace-warn "-march=native " ""` to the package's `postPatch`, stripping the flag while keeping the architecture-gated `-msse*` / `-march=armv8-a` baseline flags the CMakeLists already sets.
- **Prevention rule:** When packaging a third-party CMake project for the Nix/cachix workspace, grep its `CMakeLists.txt` for `native` and strip `-march=native`/`-mtune=native` in `postPatch`. Never let host-detected ISA flags into a cached derivation.

### ROS `package.xml` "GPLv3" mapped to `gpl3Plus` instead of `gpl3Only` — 2026-06-19

- **Severity:** Low
- **Category:** Convention
- **File(s):** `software/ros_ws/third-party-packages/vikit-common/default.nix`, `software/ros_ws/third-party-packages/vikit-ros/default.nix`
- **Pattern:** Setting `meta.license = gpl3Plus` when the upstream license statement is bare "GPLv3" with no "or later" clause. `gpl3Plus` (GPL-3.0-or-later) over-states the granted rights.
- **Root cause:** Loose habit of defaulting to the `*Plus` license attribute without checking for an explicit "or later" clause.
- **Fix applied:** Changed both packages to `gpl3Only`, matching the `<license>GPLv3</license>` declared in each `package.xml` (no LICENSE file or "or later" text exists upstream).
- **Prevention rule:** Map a bare "GPLvN" / "GPL-N.0" declaration to `gpl<N>Only`; only use `gpl<N>Plus` when the source explicitly says "or (at your option) any later version".
