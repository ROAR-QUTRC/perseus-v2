# Nix Home Manager

Home manager should be used on the big-brain and medium-brains.
It shouldn't be used on team member machines - this is because it overwrites any existing configurations you've set for your programs as well as changes systemd files, and reversing these changes will be annoying at best.
It shouldn't be able to run on team member machines because it will try to build for AArch64 CPUs, and some configurations will only be possible after using the machine-setup.sh script.

## About

Home manager is responsible for all user-specific configurations.
This includes things like ~/.config/ files, systemd files (home manager can't create these, only edit them - they should be created by the machine-setup.sh script), and shell configuration (e.g. the `.bashrc` file).
All home manager files should be in `software/home-manager/` and put in their respective folder.

Home manager config files are rebuilt whenever `home-manager switch .#<USER>` is run (where `<USER>` is usually `qutrc` - look at `software/home-manager/default.nix` for the full options) - this will rebuild all of the config files.
The config files will usually be placed in ~/.config/, but some are also set to copy their configurations to system files when changed (`software/home-manager/config-files/systemd`):

```{code-block} nix
onChange = ''
      cp -f ~/.config/systemd/network/80-can.network /etc/systemd/network/80-can.network 2&>1 /dev/null
    '';
```

## File structure

All home manager configurations should be put in the appropriate subdirectory of `software/home-manager/`. If a new config can be put in an existing file, it should, but if not, then the new file must be imported in the subdirectory's `default.nix`, and named appropriately.
