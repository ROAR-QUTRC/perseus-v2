final: prev:
let
  build-cachix-script =
    name:
    prev.runCommandLocal name { nativeBuildInputs = with prev; [ makeWrapper ]; } ''
      makeWrapper ${./cachix/${name}.sh} $out/bin/${name} \
        --prefix PATH : ${
          with prev;
          lib.makeBinPath [
            cachix
            jq
          ]
        }
    '';
  build-wrapped-script =
    name: deps:
    prev.runCommandLocal name { nativeBuildInputs = with prev; [ makeWrapper ]; } ''
      makeWrapper ${./${name}.sh} $out/bin/${name} \
        --prefix PATH : ${prev.lib.makeBinPath deps}
    '';
in
{
  scripts = {
    cachix = {
      build = build-cachix-script "build";
      shell = build-cachix-script "shell";
      all = build-cachix-script "all";
      docs-shell = build-cachix-script "docs-shell";
    };
    clean = build-wrapped-script "clean" (with prev; [ git ]);
    vcan-setup = build-wrapped-script "vcan-setup" (
      with prev;
      [
        iproute2
        kmod
      ]
    );
  };
}
