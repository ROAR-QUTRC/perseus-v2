{ pkgs, ... }:
{
  imports = [
    ./neovim.nix
  ];
  home.packages = with pkgs; [
    nano
    vim
  ];
}
