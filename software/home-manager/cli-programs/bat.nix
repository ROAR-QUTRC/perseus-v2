{ pkgs, ... }:
{
  programs.bat = {
    enable = true;
    package = pkgs.unstable.bat;
    extraPackages = with pkgs.bat-extras; [
      batdiff
      batman
      batgrep
      prettybat
    ];
  };
  home.shellAliases = {
    # bat extra tools
    man = "batman";
    bgrp = "batgrep";
  };
}
