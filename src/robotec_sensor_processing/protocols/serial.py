# -*- coding: utf-8 -*-
""" Class Serial

The class is the wrapper around the pyserial class which is dedicated to transfer 
data through a specified serial port.
"""

from robotec_sensor_processing.protocols.protocol import Protocol
from serial import Serial as PySerial

class Serial(Protocol):
    def __init__(self, port, baudrate):
        # Call the superclass'es constructor
        super().__init__(port, baudrate)

        # Create an instance of the Serial class
        self._serial = PySerial()

    """
    Update parameters of the serial connection
    """
    def updateConnectionParameters(self):
        self._serial.port = self._port
        self._serial.baudrate = self._baudrate

    """
    Opening the specified serial port to read/write data. The function doesn't 
    return anything nor getting anything as parameters. The serial port instance
    is checked whether it's opened or not (raising an exception if the latter)
    """
    def startConnection(self):
        # Update connection parameters
        self.updateConnectionParameters()

        # Open the port
        self._serial.open()

        # Check if the serial port was opened
        if not self._serial.is_open :
            raise OSError("The port was not opened!")

    """
    Closing the port.
    """
    def stopConnection(self):
        # Close the port after the use
        self._serial.close()

    """
    Sending data to the receiver through opened serial port connection. Raising
    an exception if the data is empty.
    """
    def writeData(self, data):
        # Check if command is correct
        if len(data) == 0:
            raise ValueError("Data to send is empty!")

        self._serial.write(data)

    """
    Reading data from the serial port (only data of given length is returned).
    """
    def readData(self, size = 0):
        # Read data from the serial port (only the amount of bits which is specified in "size" parameter or all the data if the size is 0)
        if size == 0:
            data = self._serial.read()
        else:
            data = self._serial.read(size)

        return data
