{ pkgs, ... }:
{
  # install font for p10k
  home.packages = with pkgs; [ meslo-lgs-nf ];

  programs.zsh = {
    enable = true;

    autocd = true;
    autosuggestion.enable = true;
    syntaxHighlighting.enable = true;

    # zsh plugin config goes here
    localVariables = {
    };

    oh-my-zsh = {
      enable = true;
      plugins = [
        "sudo"
      ];
      # theme doesn't really matter since we're using p10k, though
      theme = "eastwood";

      extraConfig = ''
        COMPLETION_WAITING_DOTS="true"
      '';
    };

    initExtra = ''
      eval "$(starship init zsh)"
    '';

  };
}
