# Nix Home Manager

Home manager should be used on the big-brain and medium-brains. It shouldn't be used on team member machines - this is because it overwrites any existing configurations you've set for your programs as well as changes systemd files, and reversing these changes will be annoying at best. It shouldn't be able to run on team member machines because it will try to build for AArch64 CPUs, and some configurations will only be possible if run after using the machine-setup.sh script.

Home manager is responsible for all user-specific configurations. This includes things like ~/.config/ files, systemd files (home manager can't create these, only edit them - they should be created by the machine-setup.sh script), and shell configuration (e.g. the `.bashrc` file). All home manager .nix files should be in `software/home-manager/` and put in their respective folder.

Home manager configs are rebuilt whenever `WHAT COMMAND` is run - this will rebuild all of the configs and set them - in ~/.config/, but some are also set to copy their configurations to system files when changed (`software/home-manager/config-files/systemd`):

```
onChange = ''
      cp -f ~/.config/systemd/network/80-can.network /etc/systemd/network/80-can.network 2&>1 /dev/null
      printf '\033[0;33m'
      echo "------------------------------"
      echo "Networking rules updated. Run \"sudo systemctl restart systemd-networkd\" for the changes to apply."
      echo "------------------------------"
      printf '\033[0m' # clear text formatting
    '';
```
