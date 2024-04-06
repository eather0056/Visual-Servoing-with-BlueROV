#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64

class DepthFromPressure:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('depth_from_pressure_node', anonymous=True)
        
        # Create a publisher for the calculated depth
        self.depth_publisher = rospy.Publisher('/br5/calculated_depth', Float64, queue_size=10)
        
        # Subscribe to the water pressure topic
        self.pressure_subscriber = rospy.Subscriber('/br5/mavros/imu/water_pressure', FluidPressure, self.pressure_callback)
        
        # Constants for depth calculation
        self.rho = 1000.0  # Density of the fluid (kg/m^3) for fresh water
        self.g = 9.80665  # Acceleration due to gravity (m/s^2)
        self.atmospheric_pressure = 101325  # Atmospheric pressure in Pascals

    def pressure_callback(self, data):
        # Calculate the depth based on the pressure reading
        pressure = data.fluid_pressure  # Pressure reading in Pascals
        depth = (pressure - self.atmospheric_pressure) / (self.rho * self.g)
        
        # Publish the calculated depth
        self.depth_publisher.publish(Float64(depth))

if __name__ == '__main__':
    try:
        DepthFromPressure()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
