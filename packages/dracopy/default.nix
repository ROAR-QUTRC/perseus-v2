{
  lib,
  buildPythonPackage,
  fetchurl,
  autoPatchelfHook,
  stdenv,
  numpy,
}:

buildPythonPackage rec {
  pname = "dracopy";
  version = "2.0.0";
  format = "wheel";

  # Prebuilt manylinux wheel from PyPI. It statically links Google's Draco C++
  # library, so there's no need to build Draco (or pybind11 bindings) ourselves.
  src = fetchurl {
    url = "https://files.pythonhosted.org/packages/bd/94/59bdf4654dd9a62520696a91f281464315c3d7331859b8b8f6ccdcd25df9/dracopy-${version}-cp313-cp313-manylinux_2_24_x86_64.manylinux_2_28_x86_64.whl";
    hash = "sha256-qgjIN7EBFAmh5YB/QqvjOahFtckxYAmco91fMh6dHgI=";
  };

  nativeBuildInputs = [ autoPatchelfHook ];
  buildInputs = [ stdenv.cc.cc.lib ];
  propagatedBuildInputs = [ numpy ];

  # No pure-Python sources to import-check beyond the compiled extension module.
  pythonImportsCheck = [ "DracoPy" ];

  meta = {
    description = "Python wrapper for Google's Draco point cloud/mesh compression library";
    homepage = "https://github.com/seung-lab/DracoPy";
    license = lib.licenses.mit;
    platforms = [ "x86_64-linux" ];
  };
}
