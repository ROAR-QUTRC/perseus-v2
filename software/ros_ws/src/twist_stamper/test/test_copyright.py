import pytest

from ament_copyright.main import main


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=[".", "test"])
    assert rc == 0, "Found errors"
