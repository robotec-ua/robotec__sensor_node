# -*- coding: utf-8 -*-
""" Class GenericTemperatureSensor

The class is created to use generic temperature sensor (the class is useful for
creating more specific classes for other temperature sensors).
"""

from .device import Device

class GenericTemperatureSensor(Device):
    """
    Constructor. Delegates the parameters to the superclass and saves a "command" 
    parameter as an internal field
    """
    def __init__(self, protocol, message, topic, command):
        super().__init__(protocol, message, topic, command)

    """
    Overriden method of producing a message
    """
    def createMessage(self):
        self._message.temperature = float(self.communicate(self._command))

        return self._message()


    """
    Communicate with the device by writing/reading data to/from the
    dedicated protocol
    """
    def communicate(self, command = 0, size = 0):
        # Open the serial port
        self._protocol.startConnection()

        # Send a command to the device
        if command != 0 :
            self._protocol.writeData(command)

        # Get the data from the device
        if size == 0:
            data = self._protocol.readData()
        else : 
            data = self._protocol.readData(size)

        # Close the serial port
        self._protocol.stopConnection()

        return data