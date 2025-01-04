{ pkgs, ... }:
{
  programs.direnv = {
    enable = true;
    nix-direnv.enable = true;
    config = {
      global = {
        hide_env_diff = true;
      };
    };
  };
  programs.zsh.oh-my-zsh = {
    plugins = [
      "direnv"
    ];
  };
}
