#!/usr/bin/env python
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import rospy
from sensor_msgs.msg import FluidPressure  # Correct message type

# Initialize ROS node
rospy.init_node('water_pressure_plotter', anonymous=True)

# Create a figure for plotting
style.use('fivethirtyeight')
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

water_pressure_data = []

# Callback function for the subscriber
def callback(data):
    water_pressure = data.fluid_pressure  # Correct field for FluidPressure message
    water_pressure_data.append(water_pressure)
    if len(water_pressure_data) > 50:  # Limit the size of the data
        water_pressure_data.pop(0)

# ROS Subscriber
rospy.Subscriber("/br2/mavros/imu/water_pressure", FluidPressure, callback)  # Correct message type

# Animation function for Matplotlib
def animate(i):
    ax1.clear()
    ax1.plot(water_pressure_data)
    ax1.set_title('Real-Time Water Pressure')
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Pressure (Pa)')

ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show()
