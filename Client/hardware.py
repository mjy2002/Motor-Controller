"""
Hardware interface for the FOC Motor Controller
https://github.com/AdinAck/Motor-Controller

Adin Ackerman
"""

from threading import Lock
from itertools import chain
from serial import Serial
from serial.serialutil import SerialException
from dataclasses import dataclass
from typing import Literal


class Motor:
    """
    A Motor object, representing the communication with a single motor

    Attributes
    ----------
    ser: Serial
        Serial object
    port: str
        Serial port
    id: int
        Motor ID for multi-motor systems
    lock: Lock
        Thread lock for serial calls
    """

    def __init__(self, port: str) -> None:
        """
        Parameters
        ----------
        port: str
            Serial port to connect to
        """
        self.port = port
        self.ser = Serial(baudrate=9600)
        self.lock = Lock()

        self.connect()

    def _sendCommand(self, cmd: str) -> str:
        """
        Send a command to the motor
        *Intended for internal use only*

        Parameters
        ----------
        cmd: str
            Command to send

        Returns
        -------
        str
            Response from motor
        """
        with self.lock:
            try:
                self.ser.write(f'{cmd}\n'.encode())
                return self.ser.readline().decode().strip()
            except SerialException:
                print(f'Connection with motor {self.id} interrupted.')
                self.ser.close()
                return '0'

    def connect(self) -> None:
        """
        Establish connection with motor
        """
        self.ser.port = self.port
        self.ser.open()
        self.id = int(self._sendCommand('I'))

    @property
    def alive(self) -> bool:
        """
        Check if motor is alive

        Returns
        -------
        bool
            True if alive, False otherwise
        """
        return self.ser.is_open

    def setPIDs(self, stage: Literal['vel', 'angle'], *args: float, **kwargs: float) -> bool:
        """
        Set PID values for angle and velocity control

        Parameters
        ----------
        stage: Literal['vel', 'angle']
            Which PID stage to set
        P: float
            Proportional gain
        I: float
            Integral gain
        D: float
            Differential gain
        R: float
            Output ramp
        L: float
            Output limit
        F: float
            Low pass filter time constant

        Returns
        -------
        bool
            Confirmation
        """
        controlType = 'A' if stage == 'angle' else 'V'

        success = True
        for char, arg in chain(zip(['P', 'I', 'D', 'R', 'L', 'F'], args), kwargs.items()):
            success &= float(self._sendCommand(
                f'M{controlType}{char}{arg}')) == arg

        return success

    @property
    def position(self) -> float:
        """
        Getter for motor position
        """
        return float(self._sendCommand('MMG6'))

    @property
    def velocity(self) -> float:
        """
        Getter for motor velocity
        """
        return float(self._sendCommand('MMG5'))

    def setCurrentLimit(self, limit: float) -> bool:
        """
        Set motor current limit

        Parameters
        ----------
        limit: float
            Limit value

        Returns
        -------
        bool
            Confirmation
        """
        return float(self._sendCommand(f'MLC{limit}')) == limit

    def setVoltageLimit(self, limit: float) -> bool:
        """
        Set motor voltage limit

        Parameters
        ----------
        limit: float
            Limit value

        Returns
        -------
        bool
            Confirmation
        """
        return float(self._sendCommand(f'MLU{limit}')) == limit

    def setVelocityLimit(self, limit: float) -> bool:
        """
        Set motor velocity limit

        Parameters
        ----------
        limit: float
            Limit value

        Returns
        -------
        bool
            Confirmation
        """
        return float(self._sendCommand(f'MLV{limit}')) == limit

    def setControlMode(self, mode: Literal['torque', 'velocity', 'angle'] = 'torque') -> bool:
        """
        Set control mode

        Parameters
        ----------
        mode: str
            Control mode

            Can be one of 3 literals: 'torque', 'velocity', or 'angle'

        Returns
        -------
        bool
            Confirmation
        """
        return self._sendCommand(f'M{mode}')[:3] == mode[:3]

    def move(self, pos: float) -> bool:
        """
        Set target position

        Parameters
        ----------
        pos: float
            Target position

        Returns
        -------
        bool
            Confirmation
        """
        return float(self._sendCommand(f'M{pos}')) == pos
