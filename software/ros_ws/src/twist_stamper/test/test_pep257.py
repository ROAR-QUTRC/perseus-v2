import pytest

from ament_pep257.main import main


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=[".", "test"])
    assert rc == 0, "Found code style errors / warnings"
