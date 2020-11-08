# -*- coding: utf-8 -*-
""" Class GenericBMS

A generic abstract class for BMS's (Battery Management System). It should be
used to create derieved classes with more concrete functionality.
"""

from .device import Device
import abc

class GenericBMS(Device):
    """
    Constructor is dedicated to call the superclass's constructor and to 
    validate and use it's own essential parameters. 
    """
    def __init__(self, port, baudrate, message, topic, data_size, capacity, banks):
        super(port, baudrate, message, topic)
        setDataSize(data_size)
        setCapacity(capacity)
        setBanks(banks)

    """
    Setter for "voltage_command" variable which is used to store a 
    device-specific command to get data. It also checks whether the command 
    has a non-zero length.
    """
    def setVoltageCommand(self, command):
        #
        if len(command) == 0:
            raise ValueError("Command for getting the main voltage is empty!")

        self._voltage_command = command

    """
    Setter for "banks_command" variable which is used to store a 
    device-specific command to get data about banks' voltages. It also checks 
    whether the command has a non-zero length.
    """
    def setBanksVoltageCommand(self, command):
        # Check if the command is incorrect
        if len(command) == 0:
            raise ValueError("Command for getting banks' voltages is empty!")

        self._banks_command = command

    """
    Setter for "data_size" variable. It's used for getting a chunk of data from
    the serial port. The method throws an exception if "data_size" is incorrect
    """
    def setDataSize(self, data_size):
        # Check if the size is incorrect
        if data_size <= 1:
            raise ValueError("Size of data can't be less than 1!")

        self._data_size = data_size

    """
    Setter for "banks" parameter. The parameter is used for getting data from the
    BMS. An exception is raised when the parameter is less than 1 (incorrect). 
    """
    def setBanks(self, banks):
        #
        if banks <= 1:
            raise ValueError("Banks amount can't be less than 1!")

        self._banks = banks

    """
    Setter for "capacity" parameters. It's essential for the charge calculations.
    Raising an exception on "capacity" <= 1 (incorrect value)
    """
    def setCapacity(self, capacity):
        #
        if capacity <= 1:
            raise ValueError("Capacity can't be less than 1!")

        self._capacity = capacity

    """
    Getting the capacity
    """
    def getCapacity(self, capacity):
        return self._capacity

    """
    Abstract method of calculating the overall voltage. It differs for different 
    BMS'es.
    """
    @abc.abstractmethod
    def calculateVoltage(self):
        pass

    """
    Abstract method of calculating voltages for every bank. It differs for 
    different BMS'es.
    """
    @abc.abstractmethod
    def calculateBanksVoltage(self):
        pass

    """
    Abstract method for calculating the overall charge of battery
    """
    @abs.abstractmethod
    def calculateCharge(self, voltage):
        pass

    """
    Abstract method for creating a ROS message (contains sensor data)
    """
    @abc.abstractmethod
    def createMessage(self):
        pass