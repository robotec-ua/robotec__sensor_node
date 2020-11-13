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
    def __init__(self, port, baudrate, message, topic, command):
        super().__init__(port, baudrate, message, topic)
        self._command = command

    """
    Overriden method of producing a message
    """
    def createMessage(self):
        self._message.temperature = float(communicate(self._command))

        return self._message()