# -*- coding: utf-8 -*-
""" Class GenericDistanceSensor

Class is dedicated for interacing a generic distance sensor. It's useful to
create more specific classes for sensors using the class as the base.
"""

from .device import Device

class GenericDistanceSensor(Device):
    """
    Constructor. Delegates the parameters to the superclass and saves a "command" 
    parameter as an internal field
    """
    def __init__(self, port, baudrate, message, topic, command):
        super(port, baudrate, message, topic)
        self._command = command

    """
    Overriden method for producing a ROS message
    """
    def createMessage(self):
        self._message.range = float(communicate(self._command))

        return self._message()