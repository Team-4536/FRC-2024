from pathlib import Path

import pytest
from pyfrc.test_support.pytest_plugin import PyFrcPlugin

from robot import Robot


def pytest_runtest_setup(item: pytest.Function) -> None:
    item.config.pluginmanager.register(
        PyFrcPlugin(Robot, Path("robot.py"), isolated=True, init_timeout=0.0),
    )
