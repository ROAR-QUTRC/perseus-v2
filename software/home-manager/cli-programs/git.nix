{ pkgs, ... }:
{
  programs.git = {
    enable = true;
    settings = {
      diff = {
        color-moved = "default";
      };
      merge = {
        # https://dandavison.github.io/delta/merge-conflicts.html
        conflict-style = "zdiff3";
      };
    };
  };
  programs.delta = {
    enable = true;
    enableGitIntegration = true;
    options = {
      # https://dandavison.github.io/delta/hyperlinks.html
      hyperlinks = true;
      hyperlinks-file-link-format = "vscode://file/{path}:{line}";
      navigate = true;
      side-by-side = true;
      line-numbers = true;
      true-color = "always";
    };
  };
  programs.gh = {
    enable = true;
    package = pkgs.unstable.gh;
  };
  programs.zsh.oh-my-zsh.plugins = [ "git" ];
}
