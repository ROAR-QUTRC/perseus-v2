{
  lib,
  buildPythonPackage,
  fetchPypi,
  colcon-core,
  scantree,
  six,
  setuptools,
}:

buildPythonPackage rec {
  pname = "colcon-clean";
  version = "0.2.1";
  pyproject = true;

  src = fetchPypi {
    inherit pname version;
    hash = "sha256-8rvyck24SxIhhP9AKiR7h1jY9pLJ8yulOAH2nabc61Q=";
  };

  build-system = [ setuptools ];

  dependencies = [
    colcon-core
    scantree
  ];

  nativeCheckInputs = [ six ];

  meta = with lib; {
    description = "Extension for colcon to clean package workspaces.";
    homepage = "https://colcon.readthedocs.io";
    license = licenses.asl20;
    # maintainers = with maintainers; [ ];
  };
}
