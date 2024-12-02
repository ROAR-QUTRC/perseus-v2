final: prev:
let
  build-cachix-script =
    name:
    build-wrapped-script name (
      with prev;
      [
        cachix
        jq
      ]
    );
  build-wrapped-script =
    name: deps:
    prev.runCommandLocal name { nativeBuildInputs = with prev; [ makeWrapper ]; } ''
      makeWrapper ${./${name}.sh} $out/bin/${name} \
        --prefix PATH : ${prev.lib.makeBinPath deps}
    '';
in
{
  scripts = {
    cachix-push-build = build-cachix-script "cachix-push-build";
    cachix-push-shell = build-cachix-script "cachix-push-shell";
    cachix-push = build-cachix-script "cachix-push";
    clean = build-wrapped-script "clean" (with prev; [ git ]);
  };
}
