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
from typing import Literal, Optional, Any


class Motor:
    """
    A Motor object, representing the communication with a single motor

    Attributes
    ----------
    ser: Serial
        Serial object
    port: str
        Serial port
    id: Optional[int]
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

    def _sendCommand(self, cmd: str, returnType: type) -> Any:
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
                return returnType(self.ser.readline().decode().strip())
            except ValueError:
                print(
                    f'Received data could not be parsed as {returnType}. COM may be desynced.'
                )
            except SerialException:
                print(f'Connection with motor {self.id} interrupted.')
                self.ser.close()

    def connect(self) -> None:
        """
        Establish connection with motor
        """
        self.ser.port = self.port
        self.ser.open()
        self.id = r if (r := self._sendCommand('I', int)) is not None else None

    def disconnect(self) -> None:
        """
        Disconnect from motor
        """
        self.disable()
        self.ser.close()

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

    @property
    def COMPrecision(self) -> Optional[int]:
        """
        Get COM decimal precision

        Returns
        -------
        Optional[int]
            Current COM precision
        """
        return r if (r := self._sendCommand('#', int)) is not None else None

    @property
    def enabled(self) -> Optional[bool]:
        """
        Check if motor is enabled

        Returns
        -------
        Optional[bool]
            True if enabled, False if disabled, None if unknown
        """
        return r if (r := self._sendCommand('ME', bool)) is not None else None

    @property
    def position(self) -> Optional[float]:
        """
        Getter for motor position

        Returns
        -------
        Optional[float]
            Current position, None if unknown
        """
        return r if (r := self._sendCommand('MMG6', float)) is not None else None

    @property
    def velocity(self) -> Optional[float]:
        """
        Getter for motor velocity

        Returns
        -------
        Optional[float]
            Current velocity, None if unknown
        """
        return r if (r := self._sendCommand('MMG5', float)) is not None else None

    def setCOMPrecision(self, decimals: int) -> bool:
        """
        Set number of decimals in COM output

        Parameters
        ----------
        decimals: int
            Number of decimals to use

        Returns
        -------
        bool
            Confirmation
        """
        assert 1 <= decimals <= 15, 'Decimal precision must be within the range [1,15].'

        return r == decimals if (r := self._sendCommand(f'#{decimals}', float)) is not None else False

    def enable(self) -> bool:
        """
        Enable motor

        Returns
        -------
        bool
            Confirmation
        """
        return r == 1 if (r := self._sendCommand('ME1', int)) is not None else False

    def disable(self) -> bool:
        """
        Disable motor

        Returns
        -------
        bool
            Confirmation
        """
        return r == 0 if (r := self._sendCommand('ME0', int)) is not None else False

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
        PIDType = 'A' if stage == 'angle' else 'V'

        success = True
        for char, arg in chain(zip(['P', 'I', 'D', 'R', 'L', 'F'], args), kwargs.items()):
            success &= r == arg if (
                r := self._sendCommand(f'M{PIDType}{char}{arg}', float)
            ) is not None else False

        return success

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
        return r == limit if (r := self._sendCommand(f'MLC{limit}', float)) is not None else False

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
        return r == limit if (r := self._sendCommand(f'MLU{limit}', float)) is not None else False

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
        return r == limit if (r := self._sendCommand(f'MLV{limit}', float)) is not None else False

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
        return r[:3] == mode[:3] if (r := self._sendCommand(f'M{mode}', str)) is not None else False

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
        return r == pos if (r := self._sendCommand(f'M{pos}', float)) is not None else False
