#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import FluidPressure
import matplotlib.pyplot as plt

# Global variables to store time and pressure data
timestamps = []
pressures = []

# Callback function to process messages from the topic
def pressure_callback(msg):
    global timestamps, pressures
    timestamps.append(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9)
    pressures.append(msg.fluid_pressure)

def plot_pressure():
    plt.plot(timestamps, pressures)
    plt.xlabel('Time (s)')
    plt.ylabel('Fluid Pressure (Pa)')
    plt.title('Fluid Pressure over Time')
    plt.grid(True)
    plt.show()

def listener():
    rospy.init_node('pressure_listener', anonymous=True)
    rospy.Subscriber("/br2/mavros/imu/water_pressure", FluidPressure, pressure_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
    plot_pressure()
