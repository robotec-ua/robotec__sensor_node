#!/usr/bin/env python

# -*- coding: utf-8 -*-
""" Class Node

Class Node represents a ROS program unit called "a node". It contains a logic
for connecting all the functionality together. 
"""

import roslib; roslib.load_manifest('robotec_sensor_processing')
import rospy
from sensor_msgs.msg import Range, Temperature
from robotec_msgs.msg import Battery
from robotec_sensor_processing.devices.distance_sensor import GenericDistanceSensor
from robotec_sensor_processing.devices.temperature_sensor import GenericTemperatureSensor

class Node:
    """
    Constructor creates the node, saves the parameters and creates the sensor 
    instances.
    """
    def __init__(self):
        self._sensors = []      # Array of created sensors

        # Getting ROS parameters
        self._rate = rospy.Rate(rospy.get_param("~rate", 10))
        sensor_dictionary = rospy.get_param("~sensors")

        # Create the sensor instances
        self.createSensorInstances(sensor_dictionary)

    """
    Function which creates instances of distance sensor class.
    """
    def createDistanceSensors(self, sensors):
        message = Range()       # Message for a sensor to publish
        sensor_instances = []   # List of sensor instances
        
        # Process every sensor
        for sensor in sensors :
            # Get device data
            port = sensor['port']
            baudrate = sensor['baudrate']
            topic = sensor['topic']
            command = sensor['command']

            # Get a specific info for ROS messages
            message.header.frame_id = sensor['id']
            message.field_of_view = sensor['field']

            # Create a new instance of sensor
            sensor_instances.append(GenericDistanceSensor(port, baudrate, message, topic, command))

        return sensor_instances

    """
    Function which creates instances of temperature sensor class.
    """
    def createTemperatureSensors(self, sensors):
        message = Temperature()     # ROS message instance
        sensor_instances = []       # List of sensor instances

        # Process every sensor
        for sensor in sensors :
            # Get device data
            port = sensor['port']
            baudrate = sensor['baudrate']
            topic = sensor['topic']
            command = sensor['command']

            # Get a specific info for ROS messages
            message.header.frame_id = sensor['id']

            # Create a new instance of sensor
            sensor_instances.append(GenericTemperatureSensor(port, baudrate, message, topic, command))

        return sensor_instances

    """
    Main function of creating sensor classes' instancess
    """
    def createSensorInstances(self, sensors):
        for sensor_type in sensors :
            typed_sensors = sensors[sensor_type]

            if (sensor_type == 'ultrasonic'):
                sensor_instances = self.createDistanceSensors(typed_sensors)

            if (sensor_type == 'infrared'):
                sensor_instances = self.createDistanceSensors(typed_sensors)

            if (sensor_type == 'temperature'):
                sensor_instances = self.createTemperatureSensors(typed_sensors)

            self._sensors.extend(sensor_instances)

    """
    Main loop function
    """
    def start(self):
        # Loop while there is no shutdown signal
        while not rospy.is_shutdown():
            # Publish messages from the sensors
            for sensor in self._sensors:
                sensor.publish()
            
            # Maintain the desirable loop rate
            self._rate.sleep()

def main() :
    # Initialize a ROS node
    rospy.init_node('robotec_sensor_node', anonymous=True)

    # Create an instance of Node class
    node = Node()
    node.start()

# Starting the program if it's a standalone package
if __name__ == '__main__':
    main()