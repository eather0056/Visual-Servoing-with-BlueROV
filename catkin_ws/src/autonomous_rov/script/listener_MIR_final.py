#!/usr/bin/env python3
# Importing necessary libraries
import math  # Library for mathematical functions
from os import kill  # Operating System dependent functionality
import string  # Library for string operations
import numpy as np  # Library for numerical computing
from yaml import FlowEntryToken  # YAML support library

# ROS (Robot Operating System) related imports
import rospy  # ROS Python library
import tf  # Transform library for ROS
from std_msgs.msg import Int16, Float64, Empty, Float64MultiArray, String  # ROS standard message types
from mavros_msgs.msg import OverrideRCIn  # ROS message types for MAVLink communication
from sensor_msgs.msg import Joy, Imu, FluidPressure, LaserScan  # ROS sensor message types
from mavros_msgs.srv import CommandLong  # ROS service type for MAVLink communication
from geometry_msgs.msg import Twist  # ROS message type for geometric data
import time  # Library for time-related functions
import sys  # Library providing access to some variables used or maintained by the Python interpreter
import argparse  # Library for parsing command-line arguments

import matplotlib.pyplot as plt  # Library for data visualization
import pandas as pd  # Library for data manipulation and analysis

# ---------- Global Variables ---------------

# Mode settings
set_mode = [True, False, False]  # Manual mode is initially set to True

# Initial conditions
init_a0 = True  # Flag indicating if the initial angle values are set
init_p0 = True  # Flag indicating if the initial depth value is set
arming = False  # Flag indicating if the vehicle is armed

# Angle and depth values
angle_wrt_startup = [0, 0, 0]  # Angles relative to startup position (roll, pitch, yaw)
depth_wrt_startup = 0  # Depth relative to startup position

# Control flags
enable_depth = False  # Flag for enabling depth control
enable_ping = True  # Flag for enabling ping
pinger_confidence = 0  # Confidence level of the ping signal
pinger_distance = 0  # Distance measured by the ping sensor

# Motor speed limits
Vmax_mot = 1900  # Maximum motor speed
Vmin_mot = 1100  # Minimum motor speed

# Linear/angular velocity 
u = 0  # Linear surge velocity 
v = 0  # Linear sway velocity
w = 0  # Linear heave velocity 
p = 0  # Angular roll velocity
q = 0  # Angular pitch velocity 
r = 0  # Angular heave velocity 

# Sample time
sample_time = 1 / 30  # Time between samples
t = 0  # Current time

# Lists to store data
depth_wrt_startup_values = []  # Recorded depth values relative to startup
Correction_depth_values = []  # Correction depth values
heav_data = []  # Heave data
cude_tra_data = []  # Cube trajectory data
Correction_yaw_data = []  # Correction yaw data
angle_wrt_startup_data = []  # Angle relative to startup data

# Global variables for previous state
xk_1 = 0  # Previous depth estimate
vk_1 = 0  # Previous heave (velocity) estimate
dt = 0.5  # Time step
a = 0.1  # Alpha coefficient
b = 0.005  # Beta coefficient
 
# Data Publishers
depth_pub = rospy.Publisher('/br5/depth_wrt_startup', Float64, queue_size=10)  # Publishes depth relative to startup
correction_depth_pub = rospy.Publisher('/br5/correction_depth', Float64, queue_size=10)  # Publishes correction depth
heav = rospy.Publisher('/br5/heav', Float64, queue_size=10)  # Publishes heave data
cude_tra = rospy.Publisher('/br5/cude_tra', Float64, queue_size=10)  # Publishes cube trajectory data
Correction_yaw_pu = rospy.Publisher('/br5/Correction_yaw', Float64, queue_size=10)  # Publishes correction yaw data
Correction_yaw_data_pu = rospy.Publisher('/br5/angle_wrt_startup', Float64, queue_size=10)  # Publishes angle relative to startup data


# ---------- Functions---------------

'''-----------------Step 1: Linear interpolations of PWM curve---------------------'''

def force_to_PWM(f):
    """
    This function converts the force values to PWM for the motors.
    
    Parameters:
        f (float): Force value
        
    Returns:
        PWM (float): PWM value for motor control
    """
    # PWM curve parameters for motor control
    PWM_positive_intercept = 1536
    PWM_positive_slope = 9.8
    PWM_negative_intercept = 1464
    PWM_negative_slope = 11.8762
    
    if f > 0:
        PWM = PWM_positive_intercept + PWM_positive_slope * f
    elif f < 0:
        PWM = PWM_negative_intercept + PWM_negative_slope * f
    else:
        PWM = 1500  # Neutral PWM signal for no force
    
    return PWM

'''----------------------Finish--------------------------------------------------'''

def joyCallback(data):
    """
    Callback function for processing joystick input.
    
    Parameters:
        data (sensor_msgs.msg.Joy): Joystick input data
    """
    global arming
    global set_mode
    global init_a0
    global init_p0
    global Sum_Errors_Vel
    global Sum_Errors_angle_yaw
    global Sum_Errors_depth

    # Joystick buttons
    btn_arm = data.buttons[7]  # Start button
    btn_disarm = data.buttons[6]  # Back button
    btn_manual_mode = data.buttons[3]  # Y button
    btn_automatic_mode = data.buttons[2]  # X button
    btn_corrected_mode = data.buttons[0]  # A button

    # Disarming when Back button is pressed
    if btn_disarm == 1 and arming == True:
        arming = False
        armDisarm(arming)

    # Arming when Start button is pressed
    if btn_arm == 1 and arming == False:
        arming = True
        armDisarm(arming)

    # Switching between manual, auto, and correction modes
    if btn_manual_mode and not set_mode[0]:
        set_mode[0] = True
        set_mode[1] = False
        set_mode[2] = False      
        rospy.loginfo("Mode manual")
    
    if btn_automatic_mode and not set_mode[1]:
        set_mode[0] = False
        set_mode[1] = True
        set_mode[2] = False      
        rospy.loginfo("Mode automatic")
    
    if btn_corrected_mode and not set_mode[2]:
        init_a0 = True
        init_p0 = True
        
        '''For All PI and PID Controls'''
        Sum_Errors_depth = 0
        Sum_Errors_angle_yaw = 0
        '''--------Finish-------------'''
        
        set_mode[0] = False
        set_mode[1] = False
        set_mode[2] = True
        rospy.loginfo("Mode correction")


def armDisarm(armed):
    """
    Arm or disarm the motors by sending a long command service.

    Parameters:
        armed (bool): Flag indicating whether to arm (True) or disarm (False) the motors
    """
    if armed:
        rospy.wait_for_service('mavros/cmd/command')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
            armService(0, 400, 0, 1, 0, 0, 0, 0, 0, 0)
            rospy.loginfo("Arming Succeeded")
        except rospy.ServiceException as e:
            rospy.loginfo("Exception while arming")
    else:
        rospy.wait_for_service('mavros/cmd/command')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
            armService(0, 400, 0, 0, 0, 0, 0, 0, 0, 0)
            rospy.loginfo("Disarming Succeeded")
        except rospy.ServiceException as e:
            rospy.loginfo("Exception while disarming")	


def velCallback(cmd_vel):
    """
    Callback function to process velocity commands.

    Parameters:
        cmd_vel (geometry_msgs.msg.Twist): Velocity command message
    """
    global set_mode

    # Only continue if manual_mode is enabled
    if set_mode[1] or set_mode[2]:
        return

    # Extract cmd_vel message
    roll_left_right = mapValueScalSat(cmd_vel.angular.x)
    yaw_left_right = mapValueScalSat(-cmd_vel.angular.z)
    ascend_descend = mapValueScalSat(cmd_vel.linear.z)
    forward_reverse = mapValueScalSat(cmd_vel.linear.x)
    lateral_left_right = mapValueScalSat(-cmd_vel.linear.y)
    pitch_left_right = mapValueScalSat(cmd_vel.angular.y)

    setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse, lateral_left_right)


def pingerCallback(data):
    """
    Callback function to process pinger data.

    Parameters:
        data (std_msgs.msg.Float64MultiArray): Pinger data message
    """
    global pinger_confidence
    global pinger_distance

    pinger_distance = data.data[0]
    pinger_confidence = data.data[1]


def OdoCallback(data):
    """
    Callback function to process odometry data.

    Parameters:
        data (geometry_msgs.msg.Pose): Odometry data message
    """
    global angle_roll_a0
    global angle_pitch_a0
    global angle_yaw_a0
    global angle_wrt_startup
    global init_a0
    global p
    global q
    global r
    global Sum_Errors_angle_yaw

    orientation = data.orientation  # Orientation data from odometry
    angular_velocity = data.angular_velocity  # Angular velocity data from odometry

    # Extract yaw angle from quaternion orientation
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    euler = tf.transformations.euler_from_quaternion(q)
    angle_roll = euler[0]
    angle_pitch = euler[1]
    angle_yaw = euler[2]

    if init_a0:
        # Initialize angles at first execution
        angle_roll_a0 = angle_roll
        angle_pitch_a0 = angle_pitch
        angle_yaw_a0 = angle_yaw
        init_a0 = False

    # Calculate angles relative to startup position
    angle_wrt_startup[0] = ((angle_roll - angle_roll_a0 + 3.0*math.pi) % (2.0*math.pi) - math.pi) * 180 / math.pi
    angle_wrt_startup[1] = ((angle_pitch - angle_pitch_a0 + 3.0*math.pi) % (2.0*math.pi) - math.pi) * 180 / math.pi
    angle_wrt_startup[2] = ((angle_yaw - angle_yaw_a0 + 3.0*math.pi) % (2.0*math.pi) - math.pi) * 180 / math.pi

    # Publish angular orientation data
    angle = Twist()
    angle.angular.x = angle_wrt_startup[0]
    angle.angular.y = angle_wrt_startup[1]
    angle.angular.z = angle_wrt_startup[2]
    pub_angle_degre.publish(angle)

    # Extract angular velocity
    p = angular_velocity.x
    q = angular_velocity.y
    r = angular_velocity.z

    # Publish angular velocity data
    vel = Twist()
    vel.angular.x = p
    vel.angular.y = q
    vel.angular.z = r
    pub_angular_velocity.publish(vel)

    # Only continue if manual_mode is disabled
    if set_mode[0]:
        return

    '''OPTIONAL QUESTION: Implementing a Proportional controller (P) and a PI to control the yaw'''

    yaw_des = 0  # Desired yaw angle

    # PI parameters to tune for P ki_yaw = 0
    kp_yaw = 1
    ki_yaw = 0

    # Error calculation
    yaw_err = yaw_des - angle_wrt_startup[2]
    Sum_Errors_angle_yaw += yaw_err

    # PI controller & for P ki_yaw = 0
    control_yaw = kp_yaw * yaw_err + ki_yaw * Sum_Errors_angle_yaw
    Correction_yaw = force_to_PWM(control_yaw)

    # Publish Correction_yaw
    Correction_yaw_data_pu.publish(Correction_yaw)

    '''------------------------Save---------------------------------------'''
    Correction_yaw_data.append(Correction_yaw)
    angle_wrt_startup_data.append(angle_wrt_startup[2])

    # Save data to CSV file
    depth_correction_data = pd.DataFrame({
        'Correction_yaw_data': Correction_yaw_data,
        'angle_wrt_startup_data': angle_wrt_startup_data,
    })
    depth_correction_data.to_csv('/home/tihan/catkin_ws/src/autonomous_rov/script/kp_20_.csv', index=False)
    '''-----------------------------Finish-----------------------------------------------'''

    # Send PWM commands to motors with yaw correction
    setOverrideRCIN(1500, 1500, 1500, Correction_yaw, 1500, 1500)


def DvlCallback(data):
	global set_mode
	global u
	global v
	global w

	u = data.velocity.x  # Linear surge velocity
	v = data.velocity.y  # Linear sway velocity
	w = data.velocity.z  # Linear heave velocity

	Vel = Twist()
	Vel.linear.x = u
	Vel.linear.y = v
	Vel.linear.z = w
	pub_linear_velocity.publish(Vel)

'''Step 5: Creating a trajectory compatible with the ROVs dynamics''' 
def cubic_traj(t):
    z_init = 0
    z_final = 0.5
    t_final = 20

    if t < t_final:
        # Calculate coefficients only once
        a2 = 3 * (z_final - z_init) / (t_final ** 2)
        a3 = -2 * (z_final - z_init) / (t_final ** 3)

        # Compute z and z_dot directly
        z = z_init + a2 * t ** 2 + a3 * t ** 3
        z_dot = 2 * a2 * t + 3 * a3 * t ** 2
    else:
        # No need to recalculate z and z_dot if t is greater than t_final
        z = z_final
        z_dot = 0

    return z, z_dot
'''-----------------------------Finish--------------------------------------------------'''

'''Step 9: Estimating the heave from the depth measurements with an alpha-beta filter'''

def estimateHeave(xm):
	global xk_1, vk_1
    # Predict next state
	xk = xk_1 + (vk_1 * dt)
	vk = vk_1

	# Update based on measurement
	rk = xm - xk
	xk += a * rk
	vk += (b * rk) / dt

	# Update global variables for next iteration
	xk_1, vk_1 = xk, vk

	return vk  # Returning the estimated heave

'''------------------------------Finish---------------------------------------------------'''
def PressureCallback(data):
    """
    Callback function to process pressure sensor data.

    Parameters:
        data (sensor_msgs.msg.FluidPressure): Pressure sensor data message
    """
    global depth_p0
    global depth_wrt_startup
    global init_p0
    global Sum_Errors_depth
    global t

    rho = 1000.0  # Density of water in kg/m^3
    g = 9.80665  # Acceleration due to gravity in m/s^2

    # Only continue if manual_mode is disabled
    if set_mode[0]:
        return
    elif set_mode[1]:
        # Only continue if automatic_mode is enabled
        # Define an arbitrary velocity command and observe robot's velocity
        setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)
        return

    pressure = data.fluid_pressure  # Extract pressure data from sensor

    if init_p0:
        # First execution, initialize depth_p0
        depth_p0 = (pressure - 101300) / (rho * g)  # Calculate initial depth
        init_p0 = False

    # Calculate depth relative to the startup position
    depth_wrt_startup = (pressure - 101300) / (rho * g) - depth_p0

    # Setup depth servo control here
    # ...
    '''Step 3: Implementing a Proportional controller for the depth'''
    '''Step 4: Implementing a depth controller with floatability compensation'''
    '''Step 7: Proportional Integral controller (PI) for the depth'''
    '''Step 10: Implementing a PID with floatability compensation'''

    depth_des = 0.5  # Desired depth in meters

    '''Step 6: Implementing the polynomial trajectory'''
    # depth_des, depth_des_dot = cubic_traj(t)
    t = t + sample_time
    Kp_depth = 10  # Proportional gain for depth control
    Ki_depth = 0.001  # Integral gain for depth control
    Kd_depth = 1  # Derivative gain for depth control

    floatability = -0.275  # Floatability compensation in Newtons

    error = -(depth_des - depth_wrt_startup)  # Calculate depth error
    Sum_Errors_depth += error  # Accumulate depth error for integral control

    # Calculate control force using proportional and integral control Step 3: Ki_depth = 0 , floatability = 0; Step 4: floatability = !! , Ki_depth = !!
    f = Kp_depth * error + Ki_depth * Sum_Errors_depth + floatability
    # heav_data_o = estimateHeave(depth_wrt_startup)

    # Step 10: Implementing a PID with floatability compensation
    # f = Kp_depth * error + Ki_depth * Sum_Errors_depth + Kd_depth * ( depth_des_dot - estimateHeave(depth_wrt_startup) ) + floatability

    # Update Correction_depth (PWM signal for depth control)
    Correction_depth = force_to_PWM(f)

    # Publish depth_wrt_startup and Correction_depth
    depth_pub.publish(depth_wrt_startup)
    correction_depth_pub.publish(Correction_depth)

    # Publish heave data
    #heav.publish(heav_data_o)

    # Publish cubic trajectory data
    cude_tra.publish(depth_des)

    '''------------------------Save---------------------------------------'''
    # Append data to lists
    depth_wrt_startup_values.append(depth_wrt_startup)
    Correction_depth_values.append(Correction_depth)
    #heav_data.append(heav_data_o)
    cude_tra_data.append(depth_des)

    # Create DataFrame for saving data to CSV file
    depth_correction_data = pd.DataFrame({
        'Depth_wrt_startup': depth_wrt_startup_values,
        'Correction_depth': Correction_depth_values,
        #'heav_data': heav_data,
        'cude_tra_data': cude_tra_data,
    })

    # Save DataFrame to CSV file
    depth_correction_data.to_csv('/home/tihan/catkin_ws/src/autonomous_rov/script/kp_20_.csv', index=False)
    '''------------------------Save---------------------------------------'''

    '''------------------------------Finish---------------------------------------------------'''
    # Send PWM commands to motors
    Correction_depth = int(Correction_depth)
    setOverrideRCIN(1500, 1500, Correction_depth, 1500, 1500, 1500)

def mapValueScalSat(value):
    """
    Maps a value to a pulse width for servo control, scaling and saturating the value.

    Parameters:
        value (float): Input value between -1 and 1

    Returns:
        int: Pulse width value scaled and saturated between 1100 and 1900
    """
    # Correction_Vel and joy between -1 and 1
    # Scaling for publishing with setOverrideRCIN values between 1100 and 1900
    # Neutral point is 1500
    pulse_width = value * 400 + 1500

    # Saturation to ensure pulse width stays within range
    if pulse_width > 1900:
        pulse_width = 1900
    if pulse_width < 1100:
        pulse_width = 1100

    return int(pulse_width)


def setOverrideRCIN(channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward, channel_lateral):
    """
    Sets RC override channels for motor control.

    Parameters:
        channel_pitch (int): Pitch control channel
        channel_roll (int): Roll control channel
        channel_throttle (int): Throttle control channel
        channel_yaw (int): Yaw control channel
        channel_forward (int): Forward control channel
        channel_lateral (int): Lateral control channel
    """
    # This function replaces setservo for motor commands.
    # It overrides RC channels inputs and simulates motor controls.
    # Each channel manages a group of motors, not individually as servo set

    msg_override = OverrideRCIn()
    msg_override.channels[0] = np.uint(channel_pitch)       # pulseCmd[4]--> pitch
    msg_override.channels[1] = np.uint(channel_roll)        # pulseCmd[3]--> roll
    msg_override.channels[2] = np.uint(channel_throttle)    # pulseCmd[2]--> heave
    msg_override.channels[3] = np.uint(channel_yaw)        # pulseCmd[5]--> yaw
    msg_override.channels[4] = np.uint(channel_forward)     # pulseCmd[0]--> surge
    msg_override.channels[5] = np.uint(channel_lateral)     # pulseCmd[1]--> sway
    msg_override.channels[6] = 1500  # Neutral position for additional channels
    msg_override.channels[7] = 1500  # Neutral position for additional channels

    # Publish the RC override message
    pub_msg_override.publish(msg_override)



def subscriber():
	rospy.Subscriber("joy", Joy, joyCallback)
	rospy.Subscriber("cmd_vel", Twist, velCallback)
	#rospy.Subscriber("mavros/imu/data", Imu, OdoCallback)
	rospy.Subscriber("mavros/imu/water_pressure", FluidPressure, PressureCallback)
	#rospy.Subscriber("/dvl/data", DVL, DvlCallback)
	rospy.Subscriber("distance_sonar", Float64MultiArray, pingerCallback)
	rospy.spin()


if __name__ == '__main__':
	armDisarm(False)  # Not automatically disarmed at startup
	rospy.init_node('autonomous_MIR', anonymous=False)
	#rospy.init_node('autonomous_MIR', anonymous=True) # for publishing data
	pub_msg_override = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size = 10, tcp_nodelay = True)
	pub_angle_degre = rospy.Publisher('angle_degree', Twist, queue_size = 10, tcp_nodelay = True)
	pub_depth = rospy.Publisher('depth/state', Float64, queue_size = 10, tcp_nodelay = True)

	pub_angular_velocity = rospy.Publisher('angular_velocity', Twist, queue_size = 10, tcp_nodelay = True)
	pub_linear_velocity = rospy.Publisher('linear_velocity', Twist, queue_size = 10, tcp_nodelay = True)

	subscriber()

