# treefmt.nix
{ ... }:
{
  # Used to find the project root
  projectRootFile = "flake.nix";

  # enable formatters and linters
  programs.actionlint.enable = true;
  programs.clang-format.enable = true;
  programs.cmake-format.enable = true;
  programs.dos2unix.enable = true;
  programs.keep-sorted.enable = true;
  programs.ruff-check.enable = true;
  programs.ruff-format.enable = true;
  programs.shellcheck.enable = true;
  programs.shfmt.enable = true;
  # TODO: Look into enabling mypy
  programs.nixfmt.enable = true;
  programs.prettier = {
    enable = true;
    settings = {
      pluginSearchDirs = [
        "./software/web_ui"
      ];
    };
  };
  programs.taplo.enable = true;
  programs.typos.enable = true;
  programs.yamlfmt.enable = true;

  # config
  settings = {
    global = {
      excludes = [
        # build dirs
        "build/**"
        "result/**"
        # generated stuff
        "generated/**"
        "treefmt.toml"
        # docs
        "docs/figures-source/**"
        "docs/pyproject/**"
        "docs/source/_static/fonts/**"
        "docs/source/robots.txt"
        "docs/source/intersphinx/**"
        # config
        "*.rviz"
        "setup.cfg"
        # ros stuff
        "software/ros_ws/src/**/resource/**"
        # non-formattable file types
        "*.drawio"
        "*.svg"
        "*.ttf"
        "*.png"
        "*.jpg"
        "*.jpeg"
        "*.gif"
        "*.bmp"
        "*.ico"
        "*.pdf"
      ];
    };
    formatter = {
      cmake-format = {
        includes = [
          "CMakeLists.txt"
          "**/CMakeLists.txt"
          "**/*.cmake"
        ];
      };
    };
  };
}
