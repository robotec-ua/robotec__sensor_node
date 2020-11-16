# -*- coding: utf-8 -*-
""" Class Protocol


"""

import abc

class Protocol:
    def __init__(self, port, baudrate):
        self.setPort(port)
        self.setBaudrate(baudrate)

    """
    Setter for the internal field "port". Raises an exception if the "port"
    parameter is empty.
    """
    def setPort(self, port):
        # Check if the port is correct
        if len(port) == 0:
            raise ValueError("The port argument is empty!")

        self._port = port

    """
    Setter for the internal field "baudrate". Raises an exception if the "baudrate"
    parameter is less than 0.
    """
    def setBaudrate(self, baudrate):
        # Check if the baudrate is correct
        if baudrate <= 1:
            raise ValueError("The baudrate is incorrect (less than 1)!")

        self._baudrate = baudrate

    """
    Opening the specified serial port to read/write data. The function doesn't 
    return anything nor getting anything as parameters. The port instance
    is checked whether it's opened or not (raising an exception if the latter)
    """
    @abc.abstractmethod
    def startConnection(self):
        pass

        """
    Closing the port.
    """
    @abc.abstractmethod
    def stopConnection(self):
        pass

    """
    Sending data to the receiver through opened serial port connection. Raising
    an exception if the data is empty.
    """
    @abc.abstractmethod
    def writeData(self, data):
        pass


    """
    Reading data from the serial port.
    """
    @abc.abstractmethod
    def readData(self, size):
        pass

    """
    Communicate with the device by writing/reading data to/from the
    dedicated serial por
    """
    @abc.abstractmethod
    def communicate(self, command, size):
        pass