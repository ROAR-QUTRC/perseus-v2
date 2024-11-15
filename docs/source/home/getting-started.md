# Getting Started

You have two options to get started with development on this project:

- **(RECOMMENDED)** Nix setup
- Standard ROS 2 setup

## Nix Setup

1. Ensure you have `curl` and `direnv` installed. For Ubuntu and other Debian-based distros, run: `sudo apt-get install -y curl direnv`
2. Run `./tooling/nix-setup.sh`
3. Restart your shell
4. `cd` to the workspace root
5. When prompted to allow the configuration, hit `y` (yes) for everything (you'll only need to do this once)
6. Wait for the downloads to finish
7. You're done!

The `nix-setup` script, in order:

- Sets up git submodules (should never be necessary, but better to have it and not need it)
- [Installs Nix](https://github.com/DeterminateSystems/nix-installer)
- Configures Nix to allow your user to configure _it_
- Adds `direnv` setup to your `.zshrc` and `.bashrc` files
- Allows `direnv` to configure based on the `.envrc` file present in this directory
- Configures `direnv` to silence absurdly long info messages on activation

After this, you'll need to restart your shell for `direnv` to take effect and set up your dev environment.

**Note: This script cannot set up direnv if you're not using `bash` or `zsh` as your shell!** You'll have to set up the hook yourself according to [its docs](https://direnv.net/docs/hook.html).

## Standard ROS 2 Setup

Follow the guide on the official [ROS 2 Humble docs](https://docs.ros.org/en/humble/Installation.html).
