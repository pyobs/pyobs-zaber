"""Smoke tests: import every public module and instantiate the driver class without
hardware, asserting it advertises the interfaces it claims.

No motor is involved: ZaberDriver only stores configuration, and the serial connection
is opened lazily inside each operation.
"""

from pyobs.interfaces import IMode, IMotion
from pyobs.modules import Module

from pyobs_zaber import ZaberModeSelector


def test_import_driver_module() -> None:
    from pyobs_zaber import zaberdriver  # noqa: F401

    assert zaberdriver.ZaberDriver is not None


def test_instantiate_mode_selector() -> None:
    selector = ZaberModeSelector(modes={"spec": 100.0, "phot": 200.0})
    assert isinstance(selector, Module)
    assert isinstance(selector, IMode)
    assert isinstance(selector, IMotion)
