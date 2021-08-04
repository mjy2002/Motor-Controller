"""
Hardware interface for the FOC Motor Controller
https://github.com/AdinAck/Motor-Controller

Adin Ackerman
"""

from threading import Lock
from serial import Serial
from serial.serialutil import SerialException


class Motor:
    """
    A Motor object, representing the communication with a single motor.

    Attributes
    ----------
    ser: Serial
        Serial object
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
        self.ser = Serial(port, 9600)
        self.lock = Lock()
        self.id = int(self._sendCommand('I'))

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
        with self.lock:
            self.ser.open()

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

    def move(self, pos: float) -> float:
        """
        Set target position

        Parameters
        ----------
        pos: float
            Target position

        Returns
        -------
        float
            Confirmed target position
        """
        return float(self._sendCommand(f'M{pos}'))
