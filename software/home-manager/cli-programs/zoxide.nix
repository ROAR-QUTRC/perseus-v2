{ ... }:
{
  programs.zoxide.enable = true;
  home.shellAliases = {
    # cd -> zoxide
    cd = "z";
    ".." = "z ..";
  };
}
