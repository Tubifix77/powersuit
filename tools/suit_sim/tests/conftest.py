from __future__ import annotations

import pytest


@pytest.fixture(autouse=True)
def _quiet_websockets(caplog):
    """websockets logs a stack trace whenever we kill a connection on purpose,
    which several scenarios do deliberately."""
    import logging

    logging.getLogger("websockets").setLevel(logging.CRITICAL)
    yield
