# -*- coding: utf-8 -*-
""" Class DailyBMS

The class is dedicated to provide a functionality of Daily BMS 
(Battery Management System) to control battery status. 
"""

from .bms import GenericBMS

class DailyBMS(GenericBMS):
    """
    Constructor. Delegates the parameters to the superclass.
    """
    def __init__(self, port, baudrate, message, topic, data_size, capacity, banks):
        super(port, baudrate, message, topic, data_size, capacity, banks)

    """
    Calculating the overall value of voltage. The method opens
    the serial port, reads and verifies data from it, processes
    it and then closes the port.
    """
    def calculateVoltage(self):
        #
        data = bytearray(communicate(self._voltage_command, self._data_size))

        # Check if data is correct
        if len(data) != self._data_size:
            raise ValueError("Data is not correct!")

        # Calculate the main voltage
        voltage = float(((data[4]<<8)|data[5]))/10

        return voltage

    """
    Calculating voltages of the banks. The method opens
    the serial port, reads and verifies data from it, processes
    it (proessing differs from the method of processing the 
    overall voltage) and then closes the port.
    """
    def calculateBanksVoltage(self):
        voltages = []       # Array of banks' voltages
        index = 0           # Index of bank
        array_amount = self._banks / 3      # Every response contains data of 3 banks

        #
        data = communicate(self._banks_command, self._data_size * array_amount)

        # Check if data is correct
        if len(data) != self._data_size:
            raise ValueError("Data is not correct!")

        # Calculate all the voltages
        for array_index in range(0, array_amount):
            # Get voltage data array (without unused information)
            voltage_data = data[array_index * self._data_size + 5 : array_index * self._data_size + 10]

            # Read data from the given response
            for data_index in range(0, 3):
                # Get voltage from the data
                voltage = (voltage_data[data_index] << 8)
                voltage = voltage | voltage_data[data_index + 1]

                # Append a new-found voltage to the array
                voltage = float(voltage) / 1000
                voltages.append(voltage)

                # Increase the index to save the next bank's voltage
                index += 1

        return voltages

    """
    Method for calculating the overall charge of battery
    """
    def calculateCharge(self, voltage):
        charge = int((self._message.voltage-26.1)/0.036)

        return charge
        
    """
    Create a message to publish
    """       
    def createMessage(self):
        self._message.voltage = calculateVoltage()
        self._message.charge = calculateCharge(self._message.voltage)
        self._message.banks_voltage = calculateBanksVoltage()

        return self._message