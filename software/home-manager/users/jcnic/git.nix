{ ... }:
{
  programs.git = {
    userName = "James N";
    userEmail = "59348282+RandomSpaceship@users.noreply.github.com";
    extraConfig = {
      pull.rebase = false;
    };
    aliases = {
      gud = "reset --hard origin/main"; # spellchecker:disable-line
    };
  };
}
