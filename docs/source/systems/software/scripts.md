# Scripts

## autocomplete.sh

For what this script actually does, run `echo $(mk-workspace-shell-setup)`, but it pretty much does what it says in the .envrc file (shown everytime you enter direnv - whenever you go into the perseus repo), setting up autocomplete for ros and python.

## clean.sh

This script will be run from the command `nix run .#clean` and shouldn't need to be modified. That said, it removes any built, installed, or generated files and basically resets the repo back to when it was first installed.

## machine-setup.sh

This script is designed to be run on the big-brain to setup any config that the nix home manager can't do itself. Currently, the script:

1. Sets the default shell to be z-shell (to give us autocomplete)
2. Sets up nix files to be accessible only by sudo (for security purposes)
3. Creates network rule files (can't create these without sudo, but home-manager can edit them without sudo)
4. Sets the CAN network send buffer length to be 128 (there was a buffer issue when running E&C)

## nix-packages.sh

This script should be run whenever the ros2 package dependencies (any package.xml files in the ros_ws folder) have been changed. It updates the nix packaging with the new dependencies and automatically commits the changes using git (use the `--no-commit` flag to not do this)

## nix-setup.sh

This script should be run on every device that wants to run the perseus repo (members, big-brain, medium-brain, etc). Currently, the script:

1. Installs Nix package manager (see the Nix Basics page)
2. Adds binary caches (and keys) to the trusted substituters so your computer can copy them instead of compiling them itself
3. Disables the dirty git tree warning (which will run _everytime_ you have local staged changes and try to build the flake)
4. Adds the direnv hooks to your shell (enables direnv in your shell, so you have the environment variables needed)
5. Hides the massive wall of text displaying the environment variables on startup/reload

:::{note}
The nix-setup script can't be run on NixOS - you'll have to set these manually in your configuration.nix file (and your home.nix home-manager file), but if you're using NixOS, you should have no trouble doing this
:::

## update.sh

This script will update the nix flake packages to their latest version and then run the formatter. It should be run frequently to ensure that the nixpkgs are up to date (you can check if it will actually update the nixpkgs by looking at the commits on the nix-ros-overlay repo [here](https://github.com/lopsided98/nix-ros-overlay)).

## vcan-setup.sh

This script can be run if you need to test CAN software on your computer without booting up Perseus. It sets up a virtual CAN network, so instead of physically connecting into the CAN network like on the big-brain, you can run Perseus ros2 scripts to make sure they work together.
