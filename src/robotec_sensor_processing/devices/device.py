# -*- coding: utf-8 -*-
""" Class Device

The class is representing a connection to a device which has an assigned port.
It sholud be used as a base class for more concrete classes.
"""

import rospy
import abc

class Device:
    """
    Constructor takes only port and baudrate as parameters and creates an
    instance of Serial class.
    """
    def __init__(self, protocol, message, topic, command = 0):
        self._protocol = protocol
        self._message = message
        self._publisher = rospy.Publisher(topic, type(self._message), queue_size = 1)
        self.setCommand(command)

    """
    Abstract method for creating a ROS message (contains sensor data).
    """
    @abc.abstractmethod
    def createMessage(self):
        pass

    """
    Publishing the message (unique for every instance).
    """
    def publish(self):
        #
        self._message.header.stamp = rospy.Time.now()

        # Publishing the message
        self._publisher.publish(self._message)

    """
    Setter for the "command" field. Raise an exception if the command is 
    an empty string or array.
    """
    def setCommand(self, command):
        if len(str(command)) == 0:
            raise ValueError("Command can't be an empty string/array!")

        self._command = command