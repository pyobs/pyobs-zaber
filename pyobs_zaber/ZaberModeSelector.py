import logging
from abc import abstractmethod
from typing import Any, List, Optional

from pyobs.interfaces import IMotion
from pyobs.interfaces.IMode import IMode
from pyobs.modules import Module
from pyobs.utils.enums import MotionStatus

from zaber_motion import Units
from zaber_motion.ascii import Connection


class ZaberModeSelector(Module, IMode, IMotion):
    """Class for the Selection of Modus with a linear Motor (e.g. Spectroscopy or Photometry)."""

    __module__ = "pyobs_zaber.ZaberModeSelector"

    def __init__(
        self,
        modes: dict,
        port: str,
        speed: float = 0,
        length_unit=Units.ANGLE_DEGREES,
        speed_unit=Units.ANGULAR_VELOCITY_DEGREES_PER_SECOND,
        **kwargs: Any,
    ):
        """Creates a new ZaberModeSelector.
        Args:
            modes: dictionary of available modes in the form {name: position}
            motor: name of the motor used to set the modes
            speed: velocity of the selector movement
            length_unit: unit of the length, must be from zaber-motion.Units
            speed_unit: unit of the velocity, must be from zaber-motion.Units
        """
        Module.__init__(self, **kwargs)

        # check
        if self.comm is None:
            logging.warning("No comm module given!")

        self.modes = modes
        self.port = port
        self.speed = speed
        self.length_unit = length_unit
        self.speed_unit = speed_unit

    async def move_by(self, length) -> None:
        """
        Move Zaber motor by a given value.
        Args:
            length: value by which the motor moves
        """

        # move
        with Connection.open_serial_port(self.port) as connection:
            connection.enable_alerts()
            device = connection.detect_devices()[0]
            # TODO: raise xxx if len(device_list) is not 1 (0 -> no device found, >1 -> try to find correct one)
            axis = device.get_axis(1)
            axis.move_relative(length, self.length_unit, velocity=self.speed, velocity_unit=self.speed_unit)

    async def check_position(self) -> float:
        """
        Get the current position of the Zaber motor.
        """
        with Connection.open_serial_port(self.port) as connection:
            connection.enable_alerts()
            device = connection.detect_devices()[0]
            axis = device.get_axis(1)
            return axis.get_position(unit=self.length_unit)

    async def move_to(self, position) -> None:
        """
        Move Zaber motor to a given position.
        Args:
            position: value to which the motor moves
        """
        # move
        step = position - self.check_position()
        await self.move_by(step)

    async def list_modes(self) -> List[str]:
        """List available modes.

        Returns:
            List of available modes.
        """
        return list(self.modes.keys())

    async def set_mode(self, mode: str, **kwargs) -> None:
        """Set the current mode.

        Args:
            mode: Name of mode to set.

        Raises:
            ValueError: If an invalid mode was given.
            MoveError: If mode selector cannot be moved.
        """
        if self.get_mode() == mode:
            logging.info("Mode %s already selected.", mode)
        logging.info("Moving mode selector ...")
        await self.move_to(self.modes[mode])
        logging.info("Mode %s ready.", mode)

    async def get_mode(self) -> str:
        """Get currently set mode.

        Returns:
            Name of currently set mode.
        """
        pos_current = self.check_position()
        for mode, mode_pos in self.modes.items():
            if pos_current == mode_pos:
                return mode
        logging.warning("None of the available modes selected. Available modes are: %s", self.list_modes())
        return "undefined"

    async def init(self, **kwargs: Any) -> None:
        """Initialize device.

        Raises:
            InitError: If device could not be initialized.
        """
        logging.error("Not implemented")

    async def park(self, **kwargs: Any) -> None:
        """Park device.

        Raises:
            ParkError: If device could not be parked.
        """
        logging.error("Not implemented")

    async def get_motion_status(self, device: Optional[str] = None, **kwargs: Any) -> MotionStatus:
        """Returns current motion status.

        Args:
            device: Name of device to get status for, or None.

        Returns:
            A string from the Status enumerator.
        """
        logging.error("Not implemented")
        return MotionStatus.ERROR

    async def stop_motion(self, device: Optional[str] = None, **kwargs: Any) -> None:
        """Stop the motion.

        Args:
            device: Name of device to stop, or None for all.
        """
        logging.error("Not implemented")

    async def is_ready(self, **kwargs: Any) -> bool:
        """Returns the device is "ready", whatever that means for the specific device.

        Returns:
            Whether device is ready
        """
        logging.error("Not implemented")
        return True
