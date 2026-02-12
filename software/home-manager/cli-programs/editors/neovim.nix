{ ... }:
{
  programs.neovim = {
    enable = true;
    defaultEditor = true;
  };

  home.shellAliases = {
    n = "nvim";
  };
}
