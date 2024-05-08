#!/usr/bin/env python3
import serial
import math
import time
import rospy
from std_msgs.msg import Int16MultiArray

rospy.init_node('GPS_publisher', anonymous=True)
pub = rospy.Publisher('GPS', Int16MultiArray, queue_size=10)

gps_msg = Int16MultiArray()

def parse_nmea(sentence):
    data = sentence.split(',')
    if data[0] == '$GPRMC':
        try:
            # Latitude
            lat_degrees = float(data[3][:2])
            lat_minutes = float(data[3][2:]) / 60.0
            lat = lat_degrees + lat_minutes
            if data[4].upper() == 'S':
                lat = -lat

            # Longitude
            lon_degrees = float(data[5][:3])
            lon_minutes = float(data[5][3:]) / 60.0
            lon = lon_degrees + lon_minutes
            if data[6].upper() == 'W':
                lon = -lon

            return lat, lon
        except (IndexError, ValueError):
            pass
    return None

# Replace '/dev/ttyUSB0' with the appropriate serial port
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust baud rate if necessary

try:
    while True:
        # Read a line from the serial port
        line = ser.readline().decode().strip()
        position = parse_nmea(line)
        if position:
            gps_msg.data = [int(position[0]*1000000), int(position[1]*1000000)]
            pub.publish(gps_msg)
            print("Latitude: %f, Longitude: %f" % position)

except KeyboardInterrupt:
    ser.close()
    print("Serial port closed.")
