import logging
import numpy as np
from typing import Any, List, Optional

from pyobs.interfaces import IMotion
from pyobs.interfaces.IMode import IMode
from pyobs.modules import Module
from pyobs.utils.enums import MotionStatus

from zaber_motion import Units
from zaber_motion.ascii import Connection

steps = 100  # TODO: ATTENTION: hard coding


def sin_prof(x, v_max, s_max):
    v = v_max * np.sin(np.pi * x / s_max)
    return v


def gauss_prof(x, v_max, s_max):
    mu = s_max / 2
    sig = s_max / 6  # -> s_max/2=3*sigma
    v = v_max * np.exp(-0.5 * (x - mu) ** 2 / sig**2)
    return v


def trian_prof(x, v_max, s_max):
    if x <= s_max / 2:
        v = 2 * v_max * x / s_max
    else:
        v = 2 * v_max * (1 - x / s_max)
    return v


profiles = {"sine": sin_prof, "gauss": gauss_prof, "triangel": trian_prof}


class ZaberModeSelector(Module, IMode, IMotion):
    """Class for the Selection of Modus with a linear Motor (e.g. Spectroscopy or Photometry)."""

    __module__ = "pyobs_zaber.ZaberModeSelector"

    def __init__(
        self,
        modes: dict,
        port: str,
        speed: float = 0,
        profile: str = None,
        length_unit=Units.ANGLE_DEGREES,
        speed_unit=Units.ANGULAR_VELOCITY_DEGREES_PER_SECOND,
        system_led: bool = False,
        **kwargs: Any,
    ):
        """Creates a new ZaberModeSelector.
        Args:
            modes: dictionary of available modes in the form {name: position}
            motor: name of the motor used to set the modes
            speed: velocity of the selector movement
            profile: optional velocity profile in order to prevent motor from abrupt start and end
                     available are 'gauss', 'sine' and 'triangel'
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
        self.profile = profile
        self.length_unit = length_unit
        self.speed_unit = speed_unit
        self.enable_led(system_led)

    async def move_by(self, length, speed=None) -> None:
        """
        Move Zaber motor by a given value.
        Args:
            length: value by which the motor moves
            speed: velocity at which the motor moves
        """
        if speed is None:
            speed = self.speed

        # move
        with Connection.open_serial_port(self.port) as connection:
            connection.enable_alerts()
            device = connection.detect_devices()[0]
            # TODO: raise xxx if len(device_list) is not 1 (0 -> no device found, >1 -> try to find correct one)
            axis = device.get_axis(1)
            axis.move_relative(length, self.length_unit, velocity=speed, velocity_unit=self.speed_unit)

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
        with Connection.open_serial_port(self.port) as connection:
            connection.enable_alerts()
            device = connection.detect_devices()[0]
            axis = device.get_axis(1)
            await axis.move_absolute_async(
                position, self.length_unit, velocity=self.speed, velocity_unit=self.speed_unit
            )

    async def profile_move_to(self, position, profile) -> None:
        """
        Move Zaber motor to a given position, where the velocity follows a given profile.
        Args:
            position: value to which the motor moves
            profile: velocity curve
        """
        s_max = position - self.check_position()
        s = 0
        ds = s_max / steps
        while abs(s) < s_max:
            v = (profile(s, s_max=s_max, v_max=self.speed) + profile(s + ds, s_max=s_max, v_max=self.speed)) / 2
            s += ds
            await self.move_by(ds, speed=v)

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
        available_modes = await self.list_modes()
        if mode in available_modes:
            if await self.get_mode() == mode:
                logging.info("Mode %s already selected.", mode)
            else:
                logging.info("Moving mode selector ...")
                if self.profile is None:
                    await self.move_to(self.modes[mode])
                else:
                    await self.profile_move_to(self.modes[mode], profile=self.profile)
                logging.info("Mode %s ready.", mode)
        else:
            logging.warning("Unknown mode %s. Available modes are: %s", mode, available_modes)

    async def get_mode(self) -> str:
        """Get currently set mode.

        Returns:
            Name of currently set mode.
        """
        pos_current = await self.check_position()
        for mode, mode_pos in self.modes.items():
            if round(pos_current) == mode_pos:
                return mode
        available_modes = await self.list_modes()
        logging.warning("None of the available modes selected. Available modes are: %s", available_modes)
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

    def enable_led(self, status: bool) -> None:
        """
        Turn on the motor's status LED.
        Args:
            status: True -> LED on, False -> LED off
        """
        with Connection.open_serial_port(self.port) as connection:
            connection.enable_alerts()
            device = connection.detect_devices()[0]
            device.settings.set("system.led.enable", float(status))
