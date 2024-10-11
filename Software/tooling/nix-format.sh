SCRIPT_DIR="$(dirname "$(readlink -f $0)")"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

cd "${WORKSPACE_ROOT}/ros_ws"
nix fmt --quiet
cd "${WORKSPACE_ROOT}/native"
nix fmt --quiet
cd "${WORKSPACE_ROOT}/shared"
nix fmt --quiet
cd "${WORKSPACE_ROOT}/machines"
nix fmt --quiet