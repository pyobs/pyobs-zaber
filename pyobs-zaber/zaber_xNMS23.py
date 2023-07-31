# ls /dev | grep -E 'ttyUSB|ttyACM'

from zaber_motion import Units
from zaber_motion.ascii import Connection

import logging
from typing import Any, List, Optional

from pyobs.modules import Module




class ZaberMotor(Module):
    """Class for the Selection of Modus (Spectroscopy or Photometry)."""

    __module__ = "pyobs.modules.selector"

    def __init__(
        self,
        port: str,
        basis: float,
        speed: float = 0,
        length_unit=Units.ANGLE_DEGREES,
        speed_unit=Units.ANGULAR_VELOCITY_DEGREES_PER_SECOND,
        **kwargs: Any,
    ):
        """
        Creates a new BaseMotor.

        Args:
        """
        Module.__init__(self, **kwargs)

        # check
        if self.comm is None:
            logging.warning("No comm module given!")
        self.port = port
        self.basis = basis
        self.speed = speed
        self.length_unit = length_unit
        self.speed_unit = speed_unit

    async def move_by(self, length,
                            speed=None,
                            length_unit=None,
                            speed_unit=None) -> None:
        """
        Move Zaber motor by a given value.
        Args:
            length: value by which the motor moves
            speed: velocity of this movement
            length_unit: unit of the length, must be from zaber-motion.Units
            speed_unit: unit of the velocity, must be from zaber-motion.Units
        """
        # set default values
        if speed is None:
            speed = self.speed
        if length_unit is None:
            length_unit = self.length_unit
        if speed_unit is None:
            speed_unit = self.speed_unit
        # move
        with Connection.open_serial_port(self.port) as connection:
            connection.enable_alerts()
            device = connection.detect_devices()[0] # TODO: raise xxx if len(device_list) is not 1 (0 -> no device found, >1 -> try to find correct one)
            axis = device.get_axis(1)
            axis.move_relative(length, length_unit, velocity=speed, velocity_unit=speed_unit)

    async def check_position(self, position_unit=None) -> float:
        """
        Get the current position of the Zaber motor.
        Args:
            position_unit: unit of the position, must be from zaber-motion.Units
        """
        if position_unit is None:
            position_unit = self.length_unit
        with Connection.open_serial_port(self.port) as connection:
            connection.enable_alerts()
            device = connection.detect_devices()[0]
            axis = device.get_axis(1)
            return axis.get_position(unit=position_unit)

    async def move_to(self, position,
                            speed=None,
                            length_unit=None,
                            speed_unit=None) -> None:
        """
        Move Zaber motor to a given position.
        Args:
            position: value to which the motor moves
            speed: velocity of this movement
            length_unit: unit of the position, must be from zaber-motion.Units
            speed_unit: unit of the velocity, must be from zaber-motion.Units
        """
        # set default values
        if speed is None:
            speed = self.speed
        if length_unit is None:
            length_unit = self.length_unit
        if speed_unit is None:
            speed_unit = self.speed
        # move
        step = position - self.check_position()
        await self.move_by(step, speed, length_unit, speed_unit)

    async def to_basis(self) -> None:
        """
        Motor moves back to basis.
        """
        await self.move_to(self.basis)

