{ ... }:
{
  programs.git = {
    settings.user.name = "James N";
    settings.user.email = "59348282+RandomSpaceship@users.noreply.github.com";
    settings = {
      pull.rebase = false;
    };
    aliases = {
      gud = "reset --hard origin/main"; # spellchecker:disable-line
    };
  };
}
