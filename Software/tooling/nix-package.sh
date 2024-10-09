# cd to the script location so everything searches in and goes to the right place
SCRIPT_PATH="$(dirname $0)"
cd $SCRIPT_PATH/..

OUTPUT_DIR=ros_ws/nix-packages

rm -r $OUTPUT_DIR
mkdir -p $OUTPUT_DIR
# need to keep the environment isolated so that direnv doesn't screw with ros2nix.
# -i (--ignore-environment) clears all environment variables and drops you into a shell
# --command automatically runs the following command and exits the shell once it completes
# nix run is the "better" way to do this, but it doesn't allow for complete environment isolation like the shell -i flag.
nix shell -i github:wentasah/ros2nix --command ros2nix --output-dir=$OUTPUT_DIR --output-as-nix-pkg-name --no-default --flake --distro humble $(find -name package.xml)
