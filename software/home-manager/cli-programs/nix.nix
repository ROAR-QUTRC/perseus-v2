{ pkgs, username, ... }:
{
  programs.nh = {
    enable = true;
    flake = "/home/${username}/perseus-v2";
  };
  home.packages = with pkgs; [
    nix-output-monitor
    nixd
  ];

  home.shellAliases = {
    and = "nom develop";
    nds = "nom develop -c $SHELL";
    nb = "nom build";
    nfc = "nix flake check --impure .";
    nhs = "nh home switch";
  };
}
