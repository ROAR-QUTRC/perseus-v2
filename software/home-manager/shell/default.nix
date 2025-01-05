{ ... }:
{
  imports = [
    ./bash.nix
    ./zsh.nix
  ];
  home.shellAliases = {
    # see https://askubuntu.com/questions/22037/aliases-not-available-when-using-sudo
    sudo = "sudo ";
  };
}
