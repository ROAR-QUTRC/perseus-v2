{
  pkgs,
  lib,
  config,
  ...
}:
{
  options = {
    programs.tmux.bg-color = lib.mkOption {
      type = lib.types.str;
      default = "color32"; # default to a nice blue
    };
  };
  config = {
    programs.tmux = {
      shortcut = "space";
      clock24 = true;
      terminal = "tmux-256color";
      extraConfig = ''
        set -g status-bg ${config.programs.tmux.bg-color}
      '';
      tmuxinator.enable = config.programs.tmux.enable;
      tmuxp.enable = config.programs.tmux.enable;
    };
    programs.zsh = {
      oh-my-zsh = lib.mkIf config.programs.tmux.enable {
        plugins = [
          "tmux"
        ];
      };
      initExtra = lib.mkIf config.programs.zellij.enable ''
        eval "$(zellij setup --generate-auto-start=zsh)"
      '';
    };

    programs.zellij = {
      enable = true;
      package = pkgs.unstable.zellij;
      enableBashIntegration = config.programs.bash.enable;
      enableZshIntegration = config.programs.zsh.enable;
      settings = { };
    };
  };
}
