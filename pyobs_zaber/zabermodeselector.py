import logging
from typing import Any

from pyobs.events import ModeChangedEvent
from pyobs.interfaces import (
    FitsHeaderEntry,
    IFitsHeaderBefore,
    IMode,
    IMotion,
    IReady,
    ModeCapabilities,
    ModeState,
    MotionState,
    ReadyState,
)
from pyobs.modules import Module
from pyobs.utils.enums import MotionStatus

from pyobs_zaber.zaberdriver import ZaberDriver

log = logging.getLogger(__name__)

_GROUP = "Mode"


class ZaberModeSelector(Module, IMode, IMotion, IFitsHeaderBefore):
    """Class for the Selection of Modus with a linear Motor (e.g. Spectroscopy or Photometry)."""

    __module__ = "pyobs_zaber.ZaberModeSelector"

    def __init__(
        self,
        modes: dict,
        zaber: dict,
        initial_mode: str | None = None,
        **kwargs: Any,
    ):
        """Creates a new ZaberModeSelector.
        Args:
            modes: dictionary of available modes in the form {name: position}
            zaber: keyword arguments for the underlying :class:`~pyobs_zaber.zaberdriver.ZaberDriver`, e.g.
                ``{"port": "/dev/ttyUSB0", "speed": 10000}``; pass ``{}`` to use all driver defaults
            initial_mode: mode to move to when the module is opened; if None, no move is made on startup

        Raises:
            ValueError: If initial_mode is not one of the configured modes.
        """
        Module.__init__(self, **kwargs)

        if initial_mode is not None and initial_mode not in modes:
            raise ValueError(f"Unknown initial mode '{initial_mode}'. Available modes: {list(modes.keys())}")

        self.driver = ZaberDriver(**zaber)
        self.modes = modes
        self.initial_mode = initial_mode
        self.current_mode = "undefined"

    async def open(self) -> None:
        """Open module."""
        await Module.open(self)
        await self.driver.open()

        if self._comm:
            await self.comm.register_event(ModeChangedEvent)

        await self.comm.set_capabilities(IMode, ModeCapabilities(modes={_GROUP: list(self.modes.keys())}))

        if self.initial_mode is not None:
            # set_mode publishes the mode and motion state (SLEWING -> POSITIONED) itself
            await self.set_mode(self.initial_mode)
        else:
            await self.comm.set_state(IMode, ModeState(modes={_GROUP: self.current_mode}))
            await self.comm.set_state(IMotion, MotionState(status=MotionStatus.IDLE))

        await self.comm.set_state(IReady, ReadyState(ready=True))

    async def set_mode(self, mode: str, group: str = "", **kwargs: Any) -> None:
        """Set the current mode.

        Args:
            mode: Name of mode to set.
            group: Name of the group to set the mode for (unused, single group only).

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
        await self.comm.send_event(ModeChangedEvent(_GROUP, mode))
        await self.comm.set_state(IMode, ModeState(modes={_GROUP: self.current_mode}))
        log.info("Mode %s ready.", mode)

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

    async def get_fits_header_before(
        self, namespaces: list[str] | None = None, **kwargs: Any
    ) -> dict[str, FitsHeaderEntry]:
        """Returns FITS header for the current status of this module."""
        return {"INSMODE": FitsHeaderEntry(self.current_mode, "Current instrument mode")}
