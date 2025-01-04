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
        "git"
        "sudo"
      ];
      theme = "eastwood";

      extraConfig = ''
        COMPLETION_WAITING_DOTS="true"
      '';
    };

    initExtraFirst = ''
      # Enable Powerlevel10k instant prompt. Should stay close to the top of ~/.zshrc.
      # Initialization code that may require console input (password prompts, [y/n]
      # confirmations, etc.) must go above this block; everything else may go below.
      if [[ -r "''${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-''${(%):-%n}.zsh" ]]; then
        source "''${XDG_CACHE_HOME:-$HOME/.cache}/p10k-instant-prompt-''${(%):-%n}.zsh"
      fi
    '';
    plugins = [
      # configure p10k
      {
        name = "p10k-config";
        src = ./p10k;
        file = "icons-concise.zsh";
      }
      # enable p10k
      {
        name = "powerlevel10k";
        src = pkgs.zsh-powerlevel10k;
        file = "share/zsh-powerlevel10k/powerlevel10k.zsh-theme";
      }
    ];
  };
}
