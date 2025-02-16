{
  lib,
  appimageTools,
  fetchurl,
}:
let
  pname = "groot2";
  version = "1.6.1";

  # for some reason the download has a capital G, but nothing else does...
  downloadName =
    lib.toUpper (builtins.substring 0 1 pname)
    + (builtins.substring 1 (builtins.stringLength pname) pname);
  src = fetchurl {
    url = "https://s3.us-west-1.amazonaws.com/download.behaviortree.dev/groot2_linux_installer/${downloadName}-v${version}-x86_64.AppImage";
    hash = "sha256-JUDtFKLFotTSzeXxtle15kUsLW6QoKf8BTmEY90ehUg=";
  };

  appimageContents = appimageTools.extract {
    inherit pname version src;
  };
in
appimageTools.wrapType2 {
  inherit pname version src;

  extraPkgs = pkgs: [ ];

  extraInstallCommands = ''
    install -m 444 -D ${appimageContents}/${pname}.desktop $out/share/applications/${pname}.desktop
    install -m 444 -D ${appimageContents}/usr/share/icons/hicolor/256x256/apps/${pname}.png \
      $out/share/icons/hicolor/256x256/apps/${pname}.png
  '';
}
