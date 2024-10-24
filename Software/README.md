### Broken OpenGL applications

Applications which use OpenGL (`rviz2`, `gazebo`) will fail to run on non-NixOS systems by default. You need to run them using the command `NIXPKGS_ALLOW_UNFREE=1 QT_QPA_PLATFORM=xcb QT_SCREEN_SCALE_FACTORS=1 nix run --impure github:guibou/nixGL -- COMMAND` instead, which will allow it to see the proper OpenGL drivers.

- `NIXPKGS_ALLOW_UNFREE=1` allows the use of unfree software packages such as NVIDIA drivers.
- `QT_QPA_PLATFORM=xcb` fixes QT because it likes to have strange issues no matter what.
- `QT_SCREEN_SCALE_FACTORS=1` fixes QT on HiDPI displays generally, otherwise it has some crazy flickering.
- `nix run` runs the default package in the input provided to it, that being `github:guibou/nixGL`.
- `--impure` tells `nix run` to use environment variables (`QT_QPA_PLATFORM` and `NIXPKS_ALLOW_UNFREE`).
- `--` passes further arguments to the package evaluated by `nix run`.
- `COMMAND` tells the `nixGL` tool what to run. This is what you would normally run on the shell - eg `rviz2` or `gazebo` just like normal.

For example: `NIXPKGS_ALLOW_UNFREE=1 QT_QPA_PLATFORM=xcb QT_SCREEN_SCALE_FACTORS=1 nix run --impure github:guibou/nixGL -- rviz2` will run `rviz2`.

This command _will_ take a while to evaluate the first time while it downloads the relevant drivers and packages, but that will all be cached the next time around.

