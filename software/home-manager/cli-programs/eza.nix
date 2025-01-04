{ ... }:
{
  programs.eza = {
    enable = true;
  };
  home.shellAliases = {
    # ls -> eza
    ls = "eza --icons=auto";
    la = "eza -a --icons=auto";
    ll = "eza -al --icons=auto";
    lt = "eza -RT --icons=auto --git-ignore";
    llt = "eza -alRT --icons=auto --git-ignore";
  };
}
