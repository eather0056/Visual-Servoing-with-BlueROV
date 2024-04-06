#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Initialize lists to store the time and water pressure data
times = []
water_pressures = []

# Callback function for the subscriber
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
    current_time = rospy.get_time()
    times.append(current_time)
    water_pressures.append(data.data)

    # Keep only the latest 100 data points for a rolling plot
    if len(times) > 1000:
        times.pop(0)
        water_pressures.pop(0)

# Function to update the plot
def update_plot(frame):
    plt.cla()  # Clear the current axes
    plt.plot(times, water_pressures, label='Water Pressure')
    plt.xlabel('Time (s)')
    plt.ylabel('Water Pressure')
    plt.title('Water Pressure Over Time')
    plt.legend()
    plt.tight_layout()

if __name__ == '__main__':
    rospy.init_node('water_pressure_listener', anonymous=True)
    rospy.Subscriber("/br2/mavros/imu/water_pressure", Float64, callback)

    # Set up plot for updating
    fig = plt.figure()
    ani = animation.FuncAnimation(fig, update_plot, interval=1000)
    

    # Show plot
    plt.show()

    # Spin until ctrl + c
    rospy.spin()
