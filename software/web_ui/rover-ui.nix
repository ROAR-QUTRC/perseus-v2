{
  stdenv,
  fetchYarnDeps,
  yarnConfigHook,
  nodejs_24,
  corepack_24,
  bash,
  gst_all_1,
  libnice,
  ...
}:
let
  rover-ui = stdenv.mkDerivation (finalAttrs: {
    pname = "rover-ui";
    version = "0.0.1";
    src = ./.;
    yarnOfflineCache = fetchYarnDeps {
      yarnLock = finalAttrs.src + "/yarn.lock";
      hash = "sha256-SYazzT70BsD8VUT30OkdmWbXCrryZO50IuF5D3ztg44=";
    };
    nativeBuildInputs = [
      yarnConfigHook
      nodejs_24
      corepack_24
    ];
    propagatedBuildInputs = [
      corepack_24
      nodejs_24
      libnice
    ]
    ++ (with gst_all_1; [
      gstreamer
      gst-plugins-bad
      gst-plugins-base
      gst-plugins-good
      gst-plugins-rs
    ]);

    outputs = [
      "out"
      "camera"
    ];

    buildPhase = ''
      runHook preBuild
      yarn --offline build
      # nix run searches for $out/bin/${finalAttrs.pname} and tries to run it
      # so we put a bash script there which runs the webui
      mkdir -p $out/bin
      echo "#!${bash}/bin/bash
      cd $out/build
      ${corepack_24}/bin/yarn start" > $out/bin/${finalAttrs.pname}
      chmod +x $out/bin/${finalAttrs.pname}
      mkdir -p $camera/bin
      echo "#!${bash}/bin/bash
      cd $out/build
      ${corepack_24}/bin/yarn camera" > $camera/bin/${finalAttrs.pname}
      chmod +x $camera/bin/${finalAttrs.pname}
      mkdir -p $out/build
      cp -r . $out/build
      runHook postBuild
    '';
  });
in
rover-ui
