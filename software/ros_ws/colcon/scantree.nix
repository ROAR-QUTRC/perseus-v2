{
  lib,
  buildPythonPackage,
  fetchPypi,
  versioneer,
  pathspec,
  attrs,
  six,
  pytest,
  pytest-cov,
  setuptools,
}:

buildPythonPackage rec {
  pname = "scantree";
  version = "0.0.1";
  pyproject = true;

  src = fetchPypi {
    inherit pname version;
    # hash = "sha256-Fb1cskSDsE2yxwZTYE6Oo1IumAh9t+OKuEgvBTmEwKw="; # 0.0.4
    # hash = "sha256-udWAg//F+N8t1qu9C2/eFtJ4QKg3AVPN+BVVymbxfPo="; # 0.0.2a0
    hash = "sha256-KosWPeDksvnk83+Mrz8LJlFyu/F0ER4b68eVVYGJWzk=";
  };

  build-system = [
    setuptools
    versioneer
  ];

  dependencies = [
    pathspec
    attrs
  ];

  nativeCheckInputs = [
    six
    pytest
    pytest-cov
  ];

  meta = with lib; {
    description = "Flexible recursive directory iterator: scandir meets glob(\"**\", recursive=True)";
    homepage = "https://github.com/andhus/scantree";
    license = licenses.mit;
    # maintainers = with maintainers; [ ];
  };
}
