# Third-Party Notice: libviso2

This directory contains a partial copy of libviso2 (LIBrary for ViSual Odometry 2),
vendored so `stereo_odometry` has an algorithm backend without a nixpkgs package
existing for it (tracked for follow-up: package it properly, see the PR that introduced
this directory).

- Upstream: https://github.com/srv/viso2 (`libviso2/` subdirectory), by Andreas Geiger,
  Institute of Measurement and Control Systems, Karlsruhe Institute of Technology.
- Only the files needed for stereo odometry are vendored: `matrix`, `matcher`, `triangle`,
  `filter`, `viso`, `viso_stereo`, `sse_to_neon`. Monocular/omnidirectional odometry and
  dense reconstruction were left out as `stereo_odometry` does not use them.
- Files are otherwise byte-for-byte copies (no reformatting, no house-style changes) so
  they stay diffable against upstream and are not held to this repo's coding standards,
  **except one deliberate crash fix**, clearly marked inline:
  - `matcher.cpp`, `Matcher::removeOutliers()` — Triangle's divide-and-conquer
    triangulator (`divconqrecurse` in `triangle.cpp`) recurses without bound and
    crashes via stack exhaustion (SIGSEGV) when handed many duplicate `(u,v)` points,
    which happens in practice on low-texture/degraded-tracking frames. Reproduced and
    confirmed via gdb backtrace. Patched with a dedup guard that bails out of outlier
    removal for that pass, mirroring the existing `size()<=3` early return, rather than
    patching Triangle's own recursion (a much larger, riskier surface). Search this
    directory for `perseus_vision patch` to find it.

## Licensing — read before distributing binaries built from this code

- **libviso2 core** (`matrix`, `matcher`, `filter`, `viso`, `viso_stereo`,
  `sse_to_neon`): **GPL-2.0-or-later**. Per upstream's own README: "If you distribute a
  software that uses libviso, you have to distribute it under GPL with the source code.
  Another option is to contact us to purchase a commercial license."
- **`triangle.{h,cpp}`** (Delaunay triangulation, used internally by the feature matcher):
  a separate, more restrictive license by Jonathan Richard Shewchuk — free for research
  and non-commercial use; **commercial use requires contacting the author** for
  permission. It is not GPL and not compatible with a permissive re-license.

perseus_vision as a whole is MIT-licensed. `stereo_odometry` is built as its own
library/executable (`perseus_vision_stereo_odom`, not `perseus_vision_components`)
specifically so this GPL/restricted code does not get statically linked into the
detector nodes' shared library. That keeps the rest of the package unaffected, but
**stereo_odometry's own binary is still bound by the licenses above**: before it ships
on hardware that leaves the team, or in any external release, get sign-off on the
licensing implications (GPL source-distribution obligation, Triangle's commercial-use
restriction).
