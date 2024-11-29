# Getting Started

This project is built using [Nix](https://nixos.org/), which makes getting started quite easy - all you have to do is install Nix and `direnv`, and they take care of the rest.

## Environment Setup

1. Ensure you have `curl` and `direnv` installed. For Ubuntu and other Debian-based distros, run: `sudo apt-get update && sudo apt-get install -y curl direnv`
2. Run `software/scripts/nix-setup.sh`
3. Restart your shell
4. `cd` to the repository root
5. When prompted to allow configuration, hit `y` (yes) for everything (you'll only need to do this once)
6. Wait for the downloads and builds to finish
7. You're done!

The script, in order:

- Sets up git submodules (should never be necessary, but better to have it and not need it)
- [Installs Nix](https://github.com/DeterminateSystems/nix-installer)
- Configures Nix to allow your user to configure _it_
- Adds `direnv` setup to your `.zshrc` and `.bashrc` files
- Allows `direnv` to configure based on the `.envrc` file present in this directory
- Configures `direnv` to silence absurdly long info messages on activation (this sometimes doesn't work - if you see a text wall when it activates, contact James N to have a look at it)

After this, you'll need to restart your shell for `direnv` to take effect and set up your dev environment.

:::{warning}
This script cannot set up direnv if you're not using `bash` or `zsh` as your shell!
You'll have to set up the hook yourself according to its [docs](https://direnv.net/docs/hook.html), or use `nix develop` instead (see [Developing](#developing-software)).
:::

The script is designed to be able to run multiple times if something goes wrong - it checks if the modifications it makes are already present, and if it detects them, skips that step.

### VSCode Setup

If you're using [VSCode](https://code.visualstudio.com/), open the `perseus-v2.code-workspace` workspace file (you'll need to do this every time you need)

### Other IDE Setup

In the event that you're using another editor, you probably have enough technical know-how to set it up yourself.
As long as it uses the environment variables from the `direnv` setup, it should be able to find and run everything.

## Building the software

Run `nix build`.
That's it.
No, really.
If that succeeds, that means that the entire project built successfully and you can now use it.
However, at this point, it's not available on your shell - `nix build` creates a symlink named `result` in your current working directory which links to whatever you just built.
If, instead, you want to build the software and have it available to you, run `nix shell` instead.
After this runs, you'll be dropped into a sub-shell with the built rover packages available to you.
:::{note}
:name: setup-autocomplete
Unfortunately, `nix shell` can't set up autocomplete for ROS2 commands - if you want this functionality (which you should!) either run `eval "$(mk-workspace-shell-setup)"` or `source software/scripts/autocomplete.sh` (the path for the latter assumes you're in the repository root) after entering the subshell.
:::
At this point, you can run standard ROS2 commands, and all the rover packages are available like they've been installed.

## Developing Software

`direnv` automatically makes all the tools you need available to you when you `cd` into the repo.
Like `nix shell` however, it unfortunately can't set up autocomplete, so you'll need to set it up just like [above](#setup-autocomplete) every time you enter the directory.
However, if you don't like `direnv` - or you don't want to set up autocomplete - you can alternatively run `nix develop` to be dropped into a `bash` shell with everything fully set up, including autocomplete.
In this environment, you can either run `nix build` to build the entire workspace, or you can `cd` to the ROS workspace root `software/ros_ws` and run `colcon build` just like you're developing in a standard ROS2 environment.
:::{important}
You must **always** run `colcon` inside the workspace root `software/ros_ws`, otherwise the configuration file may not apply properly.
:::
:::{note}
To the experienced ROS2 developers - you may notice the lack of a `--symlink-install` flag on the `colcon build` command - that's because it's configured using a [`colcon_defaults.yaml`](https://colcon.readthedocs.io/en/released/user/configuration.html#defaults-yaml) file present at `software/ros_ws/colcon_defaults.yaml` which adds this flag by default.
:::

### Before you start

However, before you start writing code, there's a few things you need to read through first.
The most important one is the software [architecture](project:/architecture/software.md), which goes over how all the software links together and how it's laid out.
The other document is the software [standards](project:/standards/software.md), which details the standards to which your software is expected to be written.
If your software _doesn't_ meet these standards, we unfortunately won't be able to merge your changes until you fix the issues - if code standards aren't enforced, the code **will** quickly become an un-maintainable mess, leading to another rewrite.

## Debugging

### ROS2 Nodes can't see each other on the network

Use the `talker` and `listener` nodes from the `demo_nodes_cpp` package to test this, with the talker on one device and the listener on another.
For some reason, the `ros2 multicast` commands mentioned in the standard ROS [troubleshooting guide](inv:ros#How-To-Guides/Installation-Troubleshooting) use a different port to the actual ROS2 DDS communications layer, and as such is not a reliable way to test - hence the use of `talker` and `listener` nodes.
This is almost certainly caused by firewall issues - specifically, blocking DDS discovery.
To allow ROS2 through the firewall on a system managed with `ufw`, run the following commands:

```{code-block} console
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
```

If you're on a system with a firewall _not_ managed by `ufw`, you probably already know how to do this yourself.
You need to allow UDP traffic to and from the address `224.0.0.0` with a mask of 4 bits (hence `224.0.0.0/4`), as this is the mask for [multicast addresses](https://en.wikipedia.org/wiki/Multicast_address).
:::{note}
In some cases, you may need to allow specific UDP ports through the firewall as well - if you're still having issues, let us know!
We'll help you debug it.
You can find a calculator for the ports in use [here](inv:ros#Concepts/Intermediate/About-Domain-ID) - just make sure to plug in the correct domain ID, as per the next section.
:::

### I installed ROS2 the normal way!

Firstly, you should really be using the Nix setup as it manages dependencies for you automatically.
It coexists perfectly happily with a standard ROS install, as it takes priority over your normal system-installed packages.
Secondly, you will probably probably experience `ROS_DOMAIN_ID` mismatches - this project defaults the `ROS_DOMAIN_ID` to 51 for development, and 42 for production deployment, since this ensures that packages in development can't interfere with those running on the rover.
:::{note}
If you set the `ROS_DOMAIN_ID` environment variable manually, it will be used instead of the defaults.
:::
:::{todo}
At some point, we might patch the ROS production environment so that it _sets_ the domain ID instead of defaulting - it causes issues with the use of `direnv` since the development environment sets the environment variable itself, and `direnv` uses that.
:::
