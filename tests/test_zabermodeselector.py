"""Unit tests for the mode-selection state logic in ZaberModeSelector.

Movement itself is hardware I/O and is mocked out; these cover mode validation,
short-circuiting, and position mapping.
"""

from unittest.mock import AsyncMock

import pytest

from pyobs_zaber import ZaberModeSelector

_MODES = {"spec": 100.0, "phot": 200.0}


def _make_selector() -> tuple[ZaberModeSelector, AsyncMock]:
    selector = ZaberModeSelector(modes=_MODES)
    move_to = AsyncMock()
    selector.driver.move_to = move_to  # type: ignore[method-assign]
    return selector, move_to


def test_constructor_stores_modes_and_driver() -> None:
    selector = ZaberModeSelector(modes=_MODES)
    assert selector.modes == _MODES
    assert selector.current_mode == "undefined"


@pytest.mark.asyncio
async def test_set_mode_moves_to_position() -> None:
    selector, move_to = _make_selector()
    await selector.set_mode("phot")
    move_to.assert_awaited_once_with(200.0)
    assert selector.current_mode == "phot"


@pytest.mark.asyncio
async def test_set_mode_unknown_mode_is_ignored() -> None:
    selector, move_to = _make_selector()
    await selector.set_mode("nope")
    move_to.assert_not_awaited()
    assert selector.current_mode == "undefined"


@pytest.mark.asyncio
async def test_set_mode_same_mode_is_noop() -> None:
    selector, move_to = _make_selector()
    await selector.set_mode("spec")
    move_to.reset_mock()
    await selector.set_mode("spec")
    move_to.assert_not_awaited()
    assert selector.current_mode == "spec"
