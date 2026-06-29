import logging
from typing import Any

from pyobs.interfaces import IMode, IMotion, IReady, MotionState, ReadyState
from pyobs.modules import Module
from pyobs.utils.enums import MotionStatus

from pyobs_zaber.zaberdriver import ZaberDriver

log = logging.getLogger(__name__)


class ZaberModeSelector(Module, IMode, IMotion):
    """Class for the Selection of Modus with a linear Motor (e.g. Spectroscopy or Photometry)."""

    __module__ = "pyobs_zaber.ZaberModeSelector"

    def __init__(
        self,
        modes: dict,
        **kwargs: Any,
    ):
        """Creates a new ZaberModeSelector.
        Args:
            modes: dictionary of available modes in the form {name: position}
        """
        Module.__init__(self, **kwargs)

        self.driver = ZaberDriver(**kwargs)
        self.modes = modes
        self.current_mode = "undefined"

    async def open(self) -> None:
        """Open module."""
        await Module.open(self)
        await self.driver.open()
        await self.comm.set_state(IReady, ReadyState(ready=True))
        await self.comm.set_state(IMotion, MotionState(status=MotionStatus.PARKED))

    async def list_modes(self, group: int = 0, **kwargs: Any) -> list[str]:
        """List available modes."""
        return list(self.modes.keys())

    async def set_mode(self, mode: str, group: int = 0, **kwargs: Any) -> None:
        """Set the current mode.

        Args:
            mode: Name of mode to set.
            group: Group number (unused, single group only).

        Raises:
            ValueError: If an invalid mode was given.
            MoveError: If mode selector cannot be moved.
        """
        if mode not in self.modes:
            log.warning("Unknown mode %s. Available modes are: %s", mode, list(self.modes.keys()))
            return
        if self.current_mode == mode:
            log.info("Mode %s already selected.", mode)
            return
        log.info("Moving mode selector ...")
        await self.comm.set_state(IMotion, MotionState(status=MotionStatus.SLEWING))
        await self.driver.move_to(self.modes[mode])
        self.current_mode = mode
        await self.comm.set_state(IMotion, MotionState(status=MotionStatus.POSITIONED))
        log.info("Mode %s ready.", mode)

    async def get_mode(self, group: int = 0, **kwargs: Any) -> str:
        """Get currently set mode."""
        return self.current_mode

    async def init(self, **kwargs: Any) -> None:
        """Initialize device."""
        await self.comm.set_state(IMotion, MotionState(status=MotionStatus.INITIALIZING))
        await self.driver.home()
        self.current_mode = "undefined"
        await self.comm.set_state(IMotion, MotionState(status=MotionStatus.IDLE))

    async def park(self, **kwargs: Any) -> None:
        """Park device."""
        await self.comm.set_state(IMotion, MotionState(status=MotionStatus.PARKING))
        await self.driver.home()
        self.current_mode = "undefined"
        await self.comm.set_state(IMotion, MotionState(status=MotionStatus.PARKED))

    async def stop_motion(self, device: str | None = None, **kwargs: Any) -> None:
        """Stop the motion."""
        await self.driver.stop()
        await self.comm.set_state(IMotion, MotionState(status=MotionStatus.IDLE))
