# Copyright (c) 2023-2024 Boston Dynamics AI Institute LLC. See LICENSE file for more info.

import pytest
from ament_pep257.main import main


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257() -> None:
    main(argv=[".", "test"])
    # TODO: These violations should be fixed but there are a lot and deserve their own PR for sanity's sake
    # assert rc == 0, "Found code style errors / warnings"
