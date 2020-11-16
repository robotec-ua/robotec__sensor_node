# -*- coding: utf-8 -*-
""" Class ProtocolFactory

This class represents Factory design pattern. It can return different types of protocols
based on input data.
"""

from robotec_sensor_processing.protocols import serial

class ProtocolFactory:
    """
    Constructor is not necessary for this class
    """
    def __init__(self):
        pass

    """
    The method returns different types of protocol instances based on the specified name.
    """
    def getProtocol(self, name, port, baudrate):
        if name == 'serial':
            protocol = serial.Serial(port, baudrate)

        else :
            raise ValueError("Unknown protocol : " + name)

        return protocol
