# -*- coding: utf-8 -*-
""" Class DeviceFactory

This class represents Factory design pattern. It can return different types of devices'
instances based on input data.
"""

from robotec_sensor_processing.devices import distance_sensor
from robotec_sensor_processing.devices import temperature_sensor

class DeviceFactory:
    TEMPERATURE_SENSOR = 0
    DISTANCE_SENSOR = 1

    """
    Constructor is not necessary for this class
    """
    def __init__(self):
        pass

    """
    The method returns different types of protocol instances based on the specified name.
    """
    def getDevice(self, code, protocol, message, topic, command = 0):
        device = ""
        
        if code == self.DISTANCE_SENSOR:
            device = distance_sensor.GenericDistanceSensor(protocol, message, topic, command)

        elif code == self.TEMPERATURE_SENSOR:
            device = temperature_sensor.GenericTemperatureSensor(protocol, message, topic, command)

        return device