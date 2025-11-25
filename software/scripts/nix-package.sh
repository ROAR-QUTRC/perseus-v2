#!/usr/bin/env bash

# If you're changing this file, make sure to read the systems/software/scripts docs

set -euo pipefail

# cd to the software directory
cd "$(git rev-parse --show-toplevel)"/software

ROS_WS="ros_ws"
OUTPUT_DIR="$ROS_WS"/nix-packages

echo "Cleaning old packaging files"
rm -r "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

echo "Running ros2nix to generate new packaging files..."
# need to keep the environment isolated so that direnv doesn't screw with ros2nix.
# -i (--ignore-environment) clears all environment variables and drops you into a shell
# --command automatically runs the following command and exits the shell once it completes
# nix run is the "better" way to do this, but it doesn't allow for complete environment isolation like the shell -i flag.
# we want word splitting here to get all the package files in one go, so disable the shellcheck warning
# shellcheck disable=SC2046
nix shell -i github:wentasah/ros2nix --command ros2nix --output-dir="$OUTPUT_DIR" --output-as-nix-pkg-name --no-default --distro jazzy $(find "$ROS_WS" -type d \( -name log -o -name build -o -name install \) -prune -o -name 'package.xml' -print)
echo "Formatting generated files"
cd "$OUTPUT_DIR"
git add .
nix fmt

if [[ $* == *--no-commit* ]]; then
  echo "Will not commit changes"
  exit 0
fi

if ! git diff --cached --quiet >/dev/null; then
  HAS_GIT_STAGING=1
  echo "Stashing staged changes"
  git stash push -S >/dev/null
fi

cleanup() {
  if [[ -v HAS_GIT_STAGING ]]; then
    echo "Restoring git stash"
    git stash pop >/dev/null
  fi
}
trap cleanup EXIT

echo "Staging changes"
git add .
echo "Committing changes"
git commit -m "chore: Update Nix packaging" >/dev/null
