# -*- coding: utf-8 -*-
""" Class Device

The class is representing a connection to a device which has an assigned port.
It sholud be a base class for more concrete classes, but can also be used as a
standalone class.
"""

import rospy
from serial import Serial
import abc

class Device:
    """
    Constructor takes only port and baudrate as parameters and creates an
    instance of Serial class.
    """
    def __init__(self, port, baudrate, message, topic):
        self._serial = Serial()
        setPort(port)
        setBaudrate(baudrate)
        self._message = message
        self._publisher = rospy.Publisher(topic, type(self._message))

    """
    Setter for the internal field "port". Raises an exception if the "port"
    parameter is empty.
    """
    def setPort(self, port):
        # Check if the port is correct
        if len(port) == 0:
            raise ValueError("The port argument is empty!")

        self._serial.port = port

    """
    Setter for the internal field "baudrate". Raises an exception if the "baudrate"
    parameter is less than 0.
    """
    def setBaudrate(self, baudrate):
        # Check if the baudrate is correct
        if baudrate <= 1:
            raise ValueError("The baudrate is incorrect (less than 1)!")

        self._serial.baudrate = baudrate

    """
    Opening the specified serial port to read/write data. The function doesn't 
    return anything nor getting anything as parameters. The serial port instance
    is checked whether it's opened or not (raising an exception if the latter)
    """
    def startConnection(self):
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
    Reading data from the serial port.
    """
    def readData(self):
        # Read data from the serial port
        data = self._serial.readline()

        return data

    """
    Reading data from the serial port (only data of given length is returned).
    """
    def readData(self, size):
        # Read data from the serial port (only the amount of bits which is specified in "size" parameter)
        data = self._serials.read(size)

        return data

    """
    Abstract method for creating a ROS message (contains sensor data)
    """
    @abc.abstractmethod
    def createMessage(self):
        pass

    """
    Publishing the message (unique for every instance)
    """
    def publish(self):
        #
        self._message.header.stamp = rospy.Time.now()

        # Publishing the message
        self._publisher.publish(self._message)

    """
    Communicate with the device by writing/reading data to/from the
    dedicated serial por
    """
    def communicate(self, command = 0, size = 0):
        # Open the serial port
        startConnection()

        # Send a command to the device
        if command == 0 :
            writeData(command)

        # Get the data from the device
        if size == 0:
            data = readData()
        else : 
            data = readData(size)

        # Close the serial port
        stopConnection()

        return data