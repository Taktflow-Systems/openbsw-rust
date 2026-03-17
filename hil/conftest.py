"""Pytest conftest — shared fixtures for HIL tests."""

import pytest
import time
from helpers.can_transport import flush_bus


@pytest.fixture(autouse=True)
def flush_between_tests():
    """Flush CAN bus before each test to avoid stale frames."""
    flush_bus(timeout=0.1)
    yield
    time.sleep(0.15)  # brief settle after each test
