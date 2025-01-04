{
  lib,
  options,
  config,
  ...
}:
{
  options = {
    programs.tmux.bg-color = lib.mkOption {
      type = lib.types.str;
      default = "color32"; # default to a nice teal
    };
  };
  config = {
    programs.tmux = {
      enable = true;
      shortcut = "space";
      clock24 = true;
      terminal = "tmux-256color";
      extraConfig = ''
        set -g status-bg ${config.programs.tmux.bg-color}
      '';
      tmuxinator.enable = true;
      tmuxp.enable = true;
    };
    programs.zsh.oh-my-zsh = {
      plugins = [
        "tmux"
      ];
    };
  };
}
