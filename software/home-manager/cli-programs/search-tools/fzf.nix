{ ... }:
{
  programs.fzf.enable = true;
  programs.zsh.oh-my-zsh = {
    plugins = [
      "fzf"
    ];
  };
}
