# cd to the Software root so it picks the flake properly
SCRIPT_DIR="$(dirname "$(readlink -f $0)")"
cd $SCRIPT_DIR/..

# push built packages
nix build .#packages.aarch64-linux.rosCore --json | jq -r '.[].outputs | to_entries[].value' | cachix push qutrc-roar
nix build .#defaultPackage.aarch64-linux --json | jq -r '.[].outputs | to_entries[].value' | cachix push qutrc-roar
nix build .#packages.aarch64-linux.roverSim --json | jq -r '.[].outputs | to_entries[].value' | cachix push qutrc-roar

# push input flakes
nix flake archive --json | jq -r '.path,(.inputs|to_entries[].value.path)' | cachix push qutrc-roar

# push dev shell environment
nix develop .#devShells.aarch64-linux.default --profile roar-devenv -c true
cachix push qutrc-roar roar-devenv

rm roar-devenv*

nix develop .#devShells.aarch64-linux.roverSim --profile roar-devenv -c true
cachix push qutrc-roar roar-devenv

rm roar-devenv*
