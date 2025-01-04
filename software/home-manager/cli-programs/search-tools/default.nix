{ ... }:
{
  imports = [
    ./fzf.nix
  ];
  programs.fd.enable = true;
  programs.ripgrep.enable = true;
}
