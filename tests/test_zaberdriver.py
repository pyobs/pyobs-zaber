"""Unit tests for the non-hardware logic in ZaberDriver."""

from contextlib import asynccontextmanager
from unittest.mock import AsyncMock, MagicMock

import pytest
from zaber_motion import Units

from pyobs_zaber.zaberdriver import ZaberDriver


def test_constructor_defaults() -> None:
    driver = ZaberDriver()
    assert driver.port == "/dev/ttyUSB1"
    assert driver.speed == 10000
    assert driver.acceleration == 800
    assert driver.length_unit is Units.ANGLE_DEGREES
    assert driver.system_led is False


@pytest.mark.asyncio
async def test_move_by_defaults_to_configured_speed(monkeypatch: pytest.MonkeyPatch) -> None:
    axis = MagicMock()
    axis.move_relative_async = AsyncMock()

    @asynccontextmanager
    async def fake_axis(port):
        yield axis

    monkeypatch.setattr("pyobs_zaber.zaberdriver.zaber_axis", fake_axis)

    driver = ZaberDriver(speed=12345)
    await driver.move_by(1.0)

    axis.move_relative_async.assert_awaited_once_with(
        1.0,
        driver.length_unit,
        velocity=12345,
        velocity_unit=driver.speed_unit,
        acceleration=driver.acceleration,
        acceleration_unit=driver.acceleration_unit,
    )
