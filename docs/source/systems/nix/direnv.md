# Nix direnv

The project uses [nix-direnv](https://github.com/nix-community/nix-direnv/) to put you into a nix development shell whenever you cd into the repo.
During the setup script (`/software/scripts/nix-setup.sh`), your shell (either `bash` or `zsh`) is set to automatically apply a direnv `hook` after every command you run.
If you're in a directory without a `.envrc` file, this does nothing, but if the current directory does have one, (like in the top level of the `perseus-v2` repo), it is run.

The setup script also applies a setting to the direnv configuration to hide all the changed variables, however, due to the debian (and therefore ubuntu) repo having an outdated version of direnv, this setting doesn't work on some machines.
This results in a wall of text appearing whenever entering the `perseus-v2` repo, which can only be solved by installing direnv a different way - on the big-brain, it's been installed from source.

Since the script places you in a nix development shell, you can have most of the commands needed to build and run parts of the repo (e.g. colcon and ros2 commands to build and run ROS2 packages).
This simplifies the development process - you don't have to go into a development shell manually whenever you want to run something.
