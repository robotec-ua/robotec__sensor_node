#!/usr/bin/env python

import roslib; roslib.load_manifest('fobots_control')
import rospy
import serial
from sensor_msgs.msg import Range, Temperature, Velocities, Battery

import binascii
import time
import threading

rospy_rate = 5

us_sensor_data = Range()
ir_sensor_data = Range()
lift = Range()
temperature = Temperature()
battery = Batteries()

request_to_left_mc_node = '4'
request_to_right_mc_node = '5'
get_general_voltage = [0xA5, 0x40, 0x90, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7D] 
get_each_voltage = [0xA5, 0x40, 0x95, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x82]    
get_general_voltage = bytearray(get_general_voltage)   
get_each_voltage = bytearray(get_each_voltage)

bms_each_voltage = [0,0,0,0,0,0,0,0,0,0,0,0,0]
bms_baud = 9600
mc_node_baud = 115200

counter = 55555
time_to_check_bms = 60000
US_MIN = 20
US_MAX = 70

ser = serial.Serial('/dev/ttyUSB1', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout=0.1)
sensor_pub = rospy.Publisher('/sonars', Range, queue_size=10)
lift_pub = rospy.Publisher('/lift', Range, queue_size=10)
temperature_pub = rospy.Publisher('/temperature', Temperature, queue_size=10)
battery_pub = rospy.Publisher('/battery', Battery, queue_size=10)


def getAnswer(answer_size):
    answer = ser.read(answer_size)
    #print("response :" , answer)
    return answer

def request_and_read(_board):
    global us_sensor_data, ir_sensor_data, mc_node_baud, bms_baud
    if _board == 'mc_node_left':
        ser.close()
        ser.baudrate = 115200
        ser.open()
        ser.write(request_to_left_mc_node) 
        reading = ser.readline().decode('utf-8', errors='replace').strip()
        data = reading.split("_")
        if len(data) < 9:
            return
#        print(data)
        try: 
            #UltraSonic    
            if (((int)(data[0]) >= US_MIN) and ((int)(data[0]) <= US_MAX)):
                us_sensor_data.header.frame_id = 'front_us_sensor'
                us_sensor_data.range = (float)(data[0])/100
                us_sensor_data.header.stamp = rospy.Time.now()
                sensor_pub.publish(us_sensor_data)
            if (((int)(data[1]) >= US_MIN) and ((int)(data[1]) <= US_MAX)):
                us_sensor_data.header.frame_id = 'front_left_us_sensor'
                us_sensor_data.range = (float)(data[1])/100
                us_sensor_data.header.stamp = rospy.Time.now()
                sensor_pub.publish(us_sensor_data)
            if (((int)(data[2]) >= US_MIN) and ((int)(data[2]) <= US_MAX)):
                us_sensor_data.header.frame_id = 'front_right_us_sensor'
                us_sensor_data.range = (float)(data[2])/100
                us_sensor_data.header.stamp = rospy.Time.now()
                sensor_pub.publish(us_sensor_data)
        	# Infrared
            if (int)(data[3]) == 0:
                ir_sensor_data.header.frame_id = 'front_left_ir_sensor'
                ir_sensor_data.range = 0.01
                ir_sensor_data.header.stamp =  rospy.Time.now()
                sensor_pub.publish(ir_sensor_data)
            if (int)(data[4]) == 0:
                ir_sensor_data.header.frame_id = 'front_right_ir_sensor'
                ir_sensor_data.range = 0.01
                ir_sensor_data.header.stamp =  rospy.Time.now()
                sensor_pub.publish(ir_sensor_data)
        	# Lift
            if (int)(data[5]) == 0:
                lift.header.frame_id = 'left_ir_lift_sensor'
                lift.range = 0
                lift.header.stamp = rospy.Time.now()
                lift_pub.publish(lift)
            else:
                lift.header.frame_id = 'left_ir_lift_sensor'
                lift.range = 1
                lift.header.stamp = rospy.Time.now()
                lift_pub.publish(lift)

            if (int)(data[6]) == 0:
                lift.header.frame_id = 'left_limit_switch'
                lift.range = 0
                lift.header.stamp = rospy.Time.now()
                lift_pub.publish(lift)
            else:
                lift.header.frame_id = 'left_limit_switch'
                lift.range = 1
                lift.header.stamp = rospy.Time.now()
                lift_pub.publish(lift)
            # Temperature
            temperature.header.frame_id = 'left_limit_switch'
            temperature.temperature = float(data[8])
            temperature_pub.publish(temperature)
            battery.header.frame_id = 'left_front_BMS'
            battery.temperature = float(data[7])
            battery_pub.publish(battery)
        except UnicodeEncodeError:
            pass
    if _board == 'mc_node_right':
        ser.close()
        ser.baudrate = 115200
        ser.open() 
        ser.write(request_to_right_mc_node) 
        reading = ser.readline().decode('utf-8', errors='replace').strip()
        data = reading.split("_")
        print(data)
        if len(data) < 9:
            return
        try:                 
            #UltraSonic    
            if (((int)(data[0]) >= US_MIN) and ((int)(data[0]) <= US_MAX)):
                us_sensor_data.header.frame_id = 'back_us_sensor'
                us_sensor_data.range = (float)(data[0])/100
                us_sensor_data.header.stamp = rospy.Time.now()
                sensor_pub.publish(us_sensor_data)
            if (((int)(data[1]) >= US_MIN) and ((int)(data[1]) <= US_MAX)):
                us_sensor_data.header.frame_id = 'back_left_us_sensor'
                us_sensor_data.range = (float)(data[1])/100
                us_sensor_data.header.stamp = rospy.Time.now()
                sensor_pub.publish(us_sensor_data)
            if (((int)(data[2]) >= US_MIN) and ((int)(data[2]) <= US_MAX)):
                us_sensor_data.header.frame_id = 'back_right_us_sensor'
                us_sensor_data.range = (float)(data[2])/100
                us_sensor_data.header.stamp = rospy.Time.now()
                sensor_pub.publish(us_sensor_data)
            # Infrared
            if (int)(data[3]) == 0:
                ir_sensor_data.header.frame_id = 'back_right_ir_sensor'
                ir_sensor_data.range = 0.01
                ir_sensor_data.header.stamp =  rospy.Time.now()
                sensor_pub.publish(ir_sensor_data)
            if (int)(data[4]) == 0:
                ir_sensor_data.header.frame_id = 'back_right_ir_sensor'
                ir_sensor_data.range = 0.01
                ir_sensor_data.header.stamp =  rospy.Time.now()
                sensor_pub.publish(ir_sensor_data)

            # Lift
            if (int)(data[5]) == 0:
                lift.header.frame_id = 'right_ir_lift_sensor'
                lift.range = 0
                lift.header.stamp = rospy.Time.now()
                lift_pub.publish(lift)
            else:
                lift.header.frame_id = 'right_ir_lift_sensor'
                lift.range = 1
                lift.header.stamp = rospy.Time.now()
                lift_pub.publish(lift)

            if (int)(data[6]) == 0:
                lift.header.frame_id = 'right_limit_switch'
                lift.range = 0
                lift.header.stamp = rospy.Time.now()
                lift_pub.publish(lift)
            else:
                lift.header.frame_id = 'right_limit_switch'
                lift.range = 1
                lift.header.stamp = rospy.Time.now()
                lift_pub.publish(lift)

            # Temperature
            temperature.header.frame_id = 'limit_switch'
            temperature.temperature = float(data[8])
            temperature_pub.publish(temperature)
            battery.header.frame_id = 'right_front_BMS'
            battery.temperature = float(data[7])
            battery_pub.publish(battery)
        except UnicodeEncodeError:
            pass

def sensor_controller():
    global us_sensor_data, ir_sensor_data, counter, rospy_rate
    rospy.init_node('sensor_control', anonymous=True)
    rate = rospy.Rate(rospy_rate) 
    
    # Ultrasonic
    us_sensor_data.radiation_type = 0
    us_sensor_data.field_of_view = 0.5
    us_sensor_data.min_range = 0.30
    us_sensor_data.max_range = 3
    # Infrared
    ir_sensor_data.radiation_type = 1
    ir_sensor_data.field_of_view = 0.5
    ir_sensor_data.min_range =  0.25
    ir_sensor_data.max_range =  1
	# Lift
    lift.radiation_type = 1
    # Temperature

    # Battery
    
    print("-----------------------SENSOR START------------------")

    while not rospy.is_shutdown():
        request_and_read('mc_node_left')
        rate.sleep()
        #request_and_read('mc_node_right')
        #rate.sleep()
        counter += int(1000/rospy_rate)
        if counter >= time_to_check_bms:
            request_and_read('bms')
            counter = 0


if __name__ == '__main__':
    try:
        sensor_controller()
    except rospy.ROSInterruptException:
        pass