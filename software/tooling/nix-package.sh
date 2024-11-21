set -e

# cd to the script location so everything searches in and goes to the right place
SCRIPT_DIR="$(dirname "$(readlink -f $0)")"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
cd $WORKSPACE_ROOT

ROS_WS="$WORKSPACE_ROOT"/ros_ws
OUTPUT_DIR="$ROS_WS"/nix-packages

echo "Cleaning old packaging files"
rm -r "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

echo "Running ros2nix to generate new packaging files..."
# need to keep the environment isolated so that direnv doesn't screw with ros2nix.
# -i (--ignore-environment) clears all environment variables and drops you into a shell
# --command automatically runs the following command and exits the shell once it completes
# nix run is the "better" way to do this, but it doesn't allow for complete environment isolation like the shell -i flag.
nix shell -i github:wentasah/ros2nix --command ros2nix --output-dir="$OUTPUT_DIR" --output-as-nix-pkg-name --no-default --distro humble $(find "$ROS_WS" -name package.xml)
echo "Formatting generated files"
cd "$OUTPUT_DIR"
nix fmt --quiet

if [[ "$*" == *--no-commit* ]]; then
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
git add "$OUTPUT_DIR"
echo "Committing changes"
git commit -m "$(date +%Y-%m-%dT%H:%M:%S) Nix packaging generation" >/dev/null
