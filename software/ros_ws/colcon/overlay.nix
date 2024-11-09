final: prev:
let
  # colcon with our extensions (colcon-clean)
  # Note that we have to pass in prev.colcon, as we're overriding the final colcon package with the no-warnings version
  colconWithExtensions = final.callPackage ./colcon-with-extensions.nix { colcon = prev.colcon; };
  # colcon with output filtered with grep to suppress the BARRAGE of warnings from colcon about the ament prefix path
  # Unfortunately, they're just a side effect of the nix ros build system
  colcon = final.callPackage ./colcon-no-warnings.nix { };
  # mini package which puts COLCON_IGNORE in the output result folder
  # allows colcon build of workspace after run nix build
  colcon-ignore = final.callPackage ./colcon-ignore.nix { };
in
{
  # manually packaged python packages needed for colcon clean
  pythonPackagesExtensions = prev.pythonPackagesExtensions ++ [
    (pyFinal: pyPrev: {
      colcon-clean = pyFinal.callPackage ./clean.nix { };
      scantree = pyFinal.callPackage ./scantree.nix { };
    })
  ];

  inherit colcon colconWithExtensions colcon-ignore;
}
