#!/usr/bin/env python2
# Import necessary Python and ROS libraries for mathematical operations, 
# system operations, messaging, and transformation calculations.
import math
import numpy as np
import rospy
import tf
from std_msgs.msg import Int16, Float64, Float64MultiArray, Empty, String
from sensor_msgs.msg import Joy, Imu, FluidPressure, LaserScan
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist
import time
import sys
import argparse
import pandas as pd

# Define global variables to hold state and configuration
desired_point_vs = [0, 0]  # Desired point for visual servoing, initially zero
tracked_point_vs = [0, 0]  # Tracked point from visual servoing, initially zero

# Mode settings: manual, automatic without correction, and automatic with correction
set_mode = [True, False, False]  # Start in manual mode by default

# Initialization flags for angles and depth, plus arming state
init_a0 = True
init_p0 = True
arming = False

# Angles and depth relative to startup positions
angle_wrt_startup = [0] * 3
angle_roll_a0 = 0.0
angle_pitch_a0 = 0.0
angle_yaw_a0 = 0.0
depth_wrt_startup = 0
depth_p0 = 0

enable_depth = False  # Depth control flag
enable_ping = True    # Sonar pinging flag
pinger_confidence = 0 # Confidence in sonar ping
pinger_distance = 0   # Distance from sonar ping

# Motor velocity limits
Vmax_mot = 1900
Vmin_mot = 1100

# Linear and angular velocities, initialized to zero
u, v, w = 0, 0, 0  # Linear velocities in surge, sway, and heave
p, q, r = 0, 0, 0  # Angular velocities in roll, pitch, and yaw

cal_PWM = []
error_vs_sv = 0
vcam_vs_sv = 0
vrobot_vs_sv = 0
cal_PWM_vs_sv = 0

i = 1

# Callback function for joystick input
def joyCallback(data):
    global arming, set_mode
    global init_a0, init_p0

    # Joystick buttons mapping
    btn_arm = data.buttons[7]         # Start button for arming
    btn_disarm = data.buttons[6]      # Back button for disarming
    btn_manual_mode = data.buttons[3] # Y button for manual mode
    btn_automatic_mode = data.buttons[2] # X button for automatic mode
    btn_corrected_mode = data.buttons[0] # A button for corrected mode

    # Handle disarming
    if btn_disarm == 1 and arming:
        arming = False
        armDisarm(arming)  # Function call to handle disarming (not shown)

    # Handle arming
    if btn_arm == 1 and not arming:
        arming = True
        armDisarm(arming)  # Function call to handle arming (not shown)

    # Handle mode switching based on button presses
    # Manual mode
    if btn_manual_mode and not set_mode[0]:
        set_mode = [True, False, False]
        rospy.loginfo("Mode manual")
        
    # Automatic mode without correction
    if btn_automatic_mode and not set_mode[1]:
        set_mode = [False, True, False]
        rospy.loginfo("Mode automatic")
        
    # Automatic mode with correction
    if btn_corrected_mode and not set_mode[2]:
        init_a0, init_p0 = True, True
        # Reset sum of errors for corrected mode here
        set_mode = [False, False, True]
        rospy.loginfo("Mode correction")


def armDisarm(armed):
    """
    Sends a command to arm or disarm the vehicle's motors using the MAVROS service.

    Parameters:
    - armed (bool): True to arm the vehicle, False to disarm it.

    This function utilizes the `mavros/cmd/command` service to send a MAVLink command
    with a command code of 400 (MAV_CMD_COMPONENT_ARM_DISARM). The arm/disarm state is 
    set by the fourth parameter (1 for arm, 0 for disarm). It logs the success of the operation
    or catches and logs any service exceptions.
    """
    # Wait for the 'mavros/cmd/command' service to be available.
    rospy.wait_for_service('mavros/cmd/command')
    try:
        # Create a service proxy for sending arm/disarm commands.
        armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
        # Send the command with appropriate parameters.
        armService(0, 400, 0, 1 if armed else 0, 0, 0, 0, 0, 0, 0)
        rospy.loginfo("Arming Succeeded" if armed else "Disarming Succeeded")
    except rospy.ServiceException as e:
        rospy.loginfo("Exception during arming" if armed else "Exception during disarming")

def velCallback(cmd_vel):
    """
    Callback function for processing velocity commands based on joystick input or
    automated control scripts. This function is not executed in manual mode.

    Parameters:
    - cmd_vel (Twist): The velocity command message containing linear and angular velocities.

    The function maps the received linear and angular velocities to corresponding
    PWM values for the vehicle's motors using a custom mapping function `mapValueScalSat`.
    It then sends these PWM commands to the motors if not in manual mode.
    """
    global set_mode

    # Early return if in manual mode or automatic without correction mode.
    if set_mode[1] or set_mode[2]:
        return

    # Mapping of cmd_vel to motor PWM values.
    roll_left_right = mapValueScalSat(cmd_vel.angular.x)
    yaw_left_right = mapValueScalSat(-cmd_vel.angular.z)
    ascend_descend = mapValueScalSat(cmd_vel.linear.z)
    forward_reverse = mapValueScalSat(cmd_vel.linear.x)
    lateral_left_right = mapValueScalSat(-cmd_vel.linear.y)
    pitch_left_right = mapValueScalSat(cmd_vel.angular.y)

    # Send the mapped PWM values to the vehicle's motors.
    setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse, lateral_left_right)

def pingerCallback(data):
    """
    Callback function for processing pinger (sonar beacon) signals. It updates global variables
    for the pinger's confidence and distance based on the received data.

    Parameters:
    - data (Float64MultiArray): Contains the pinger's distance and confidence level.
    """
    global pinger_confidence, pinger_distance

    # Update the global variables with the pinger's distance and confidence from the data.
    pinger_distance = data.data[0]
    pinger_confidence = data.data[1]


def OdoCallback(data):
	global angle_roll_a0
	global angle_pitch_a0
	global angle_yaw_a0
	global angle_wrt_startup
	global init_a0
	global p
	global q
	global r

	orientation = data.orientation
	angular_velocity = data.angular_velocity

	# extraction of yaw angle
	q = [orientation.x, orientation.y, orientation.z, orientation.w]
	euler = tf.transformations.euler_from_quaternion(q)
	angle_roll = euler[0]
	angle_pitch = euler[1]
	angle_yaw = euler[2]

	if (init_a0):
		# at 1st execution, init
		angle_roll_a0 = angle_roll
		angle_pitch_a0 = angle_pitch
		angle_yaw_a0 = angle_yaw
		init_a0 = False

	angle_wrt_startup[0] = ((angle_roll - angle_roll_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
	angle_wrt_startup[1] = ((angle_pitch - angle_pitch_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
	angle_wrt_startup[2] = ((angle_yaw - angle_yaw_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
	
	angle = Twist()
	angle.angular.x = angle_wrt_startup[0]
	angle.angular.y = angle_wrt_startup[1]
	angle.angular.z = angle_wrt_startup[2]

	pub_angle_degre.publish(angle)

	# Extraction of angular velocity
	p = angular_velocity.x
	q = angular_velocity.y
	r = angular_velocity.z

	vel = Twist()
	vel.angular.x = p
	vel.angular.y = q
	vel.angular.z = r
	pub_angular_velocity.publish(vel)

	# Only continue if manual_mode is disabled
	if (set_mode[0]):
		return

	# Send PWM commands to motors
	# yaw command to be adapted using sensor feedback	
	Correction_yaw = 1500 
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

def PressureCallback(data):
	global depth_p0
	global depth_wrt_startup
	global init_p0
	rho = 1000.0 # 1025.0 for sea water
	g = 9.80665

	# Only continue if manual_mode is disabled
	if (set_mode[0]):
		return
	elif (set_mode[1]):
		# Only continue if automatic_mode is enabled
		# Define an arbitrary velocity command and observe robot's velocity
		setOverrideRCIN(1500, 1500, 1500, 1500, 1500, 1500)
		return

	pressure = data.fluid_pressure

	if (init_p0):
		# 1st execution, init
		depth_p0 = (pressure - 101300)/(rho*g)
		init_p0 = False

	depth_wrt_startup = (pressure - 101300)/(rho*g) - depth_p0

	# setup depth servo control here
	# ...

	# update Correction_depth
	Correction_depth = 1500	
	# Send PWM commands to motors
	Correction_depth = int(Correction_depth)
	setOverrideRCIN(1500, 1500, Correction_depth, 1500, 1500, 1500)

def mapValueScalSat(value):
	# Correction_Vel and joy between -1 et 1
	# scaling for publishing with setOverrideRCIN values between 1100 and 1900
	# neutral point is 1500
	pulse_width = value * 400 + 1500

	# Saturation
	if pulse_width > 1900:
		pulse_width = 1900
	if pulse_width < 1100:
		pulse_width = 1100

	return int(pulse_width)


def setOverrideRCIN(channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward, channel_lateral):
	# This function replaces setservo for motor commands.
	# It overrides Rc channels inputs and simulates motor controls.
	# In this case, each channel manages a group of motors not individually as servo set

	msg_override = OverrideRCIn()
	msg_override.channels[0] = np.uint(channel_pitch)       # pulseCmd[4]--> pitch	
	msg_override.channels[1] = np.uint(channel_roll)        # pulseCmd[3]--> roll
	msg_override.channels[2] = np.uint(channel_throttle)    # pulseCmd[2]--> heave 
	msg_override.channels[3] = np.uint( channel_yaw)        # pulseCmd[5]--> yaw
	msg_override.channels[4] = np.uint(channel_forward)     # pulseCmd[0]--> surge
	msg_override.channels[5] = np.uint(channel_lateral)     # pulseCmd[1]--> sway
	msg_override.channels[6] = 1500
	msg_override.channels[7] = 1500

	pub_msg_override.publish(msg_override)

def desiredpointcallback(data):
    """
    Callback function for updating the global desired point variable based on incoming data.

    Parameters:
    - data (Float64MultiArray): A message containing the desired point.
    """
    global desired_point_vs  # Declare use of global variable
    desired_point_vs = data.data  # Update the global variable with the new desired point

def transform(vcam, homogeneous_transform):
    """
    Transforms a velocity vector from the camera frame to another coordinate frame using a homogeneous transformation matrix.

    Parameters:
    - vcam (np.array): A velocity vector (or any vector) in the camera's coordinate frame. This vector can include
      both linear and angular components, depending on its length and the application.
    - homogeneous_transform (np.array): A 4x4 homogeneous transformation matrix that defines the transformation
      from the camera frame to the target coordinate frame. This matrix should include both rotation and translation components.

    Returns:
    - np.array: The transformed vector in the new coordinate frame.

    Note:
    - The function uses `np.matmul` for matrix multiplication, which requires `vcam` to be in a compatible shape
      for multiplication with the `homogeneous_transform` matrix. Ensure `vcam` is a column vector if necessary.
    """
    return np.matmul(homogeneous_transform, vcam)  # Perform the matrix multiplication and return the result


# def interactionMatrix(currentPoint,Z=1):
# 	return np.matrix([
# 		[-1/Z, 0, currentPoint[0]/Z, currentPoint[0]*currentPoint[1], -(1 + currentPoint[0]**2), currentPoint[1]],
# 		[0, -1/Z, currentPoint[1]/Z, 1 + currentPoint[1]**2, -currentPoint[0]*currentPoint[1], -currentPoint[0]]
# 	])
	
def interactionMatrix(currentPoint, Z=1):
    """
    Calculates the interaction matrix (L) for a point feature based on its current position in the
    image plane and its depth relative to the camera.

    The interaction matrix relates the velocity of a point in the image plane to the camera's 
    velocity in 3D space. It is a crucial element in the control law of visual servoing systems, 
    allowing for the derivation of camera velocities that minimize the error between the current 
    and desired positions of point features in the image.

    Parameters:
    - currentPoint (tuple or list): The (x, y) coordinates of the point feature in the image plane.
    - Z (float, optional): The depth of the point feature relative to the camera along the camera's
      optical axis. Defaults to 1, assuming a normalized projection for simplification or lack of depth information.

    Returns:
    - numpy.matrix: A 2x6 interaction matrix for the point feature.

    Notes:
    - The interaction matrix is derived under the assumption of a pinhole camera model and 
      small motion between consecutive images.
    """
    x, y = currentPoint  # Unpack the current position of the point in the image plane
    
    # Define the interaction matrix L for the point, a pinhole camera model.
    # This matrix is based on the partial derivatives of the image point's coordinates
    # with respect to the camera's motion parameters (translation and rotation).
    L = np.matrix([
        [-1/Z, 0, x/Z, x*y, -(1 + x**2), y],
        [0, -1/Z, y/Z, 1 + y**2, -x*y, -x]
    ])
    
    return L  # Return the interaction matrix


def trackercallback(data):
	"""
	Processes the current point data from a tracking system, computes the error between the current
	and desired points, calculates the interaction matrix, and generates velocity commands to reduce
	the error, ultimately adjusting the robot's movement towards the target.

	Parameters:
	- data: A message containing the current tracked point's position.

	This function operates within a visual servoing context, where the objective is to align a camera's
	view with a specific target view. The process involves several steps, including error computation,
	interaction matrix calculation, pseudo-inverse calculation for deriving camera velocity, and transforming
	this velocity into the robot's frame for action execution.
	"""
	global desired_point_vs  # Use the global variable to get the desired point
	global cal_PWM
	global i

	# Unpack the current tracked point from the incoming data
	current_point_vs = data.data
	i = i + 10

	# Only continue if manual_mode is disabled
	if (set_mode[0]):
		return
		
	# Compute the 2D error vector between the current and desired points
	error_x = current_point_vs[0] - desired_point_vs[0]
	error_y = current_point_vs[1] - desired_point_vs[1]
	error_vs = np.array([error_x, error_y])

	# Calculate the interaction matrices for the current and desired points, and their average
	currentL = interactionMatrix(current_point_vs)
	desiredL = interactionMatrix(desired_point_vs)
	mixedL = (currentL + desiredL) / 2
	L = currentL  # Use the current interaction matrix for further calculations
      
    # Prune degrees of freedom based on the system's control strategy; here, we focus on yaw and heave
	# Option 1: delete all degrees of freedom except yaw and heave
	L = np.delete(L, [0, 1, 3, 4], 1)

	# Options 2: delete all degrees of freedom except sway and heave
	#L = np.delete(L, [0, 3, 4, 5], 1)

	# Calculate the pseudo-inverse of the interaction matrix
	lamb = 1
	L_pseudo = np.linalg.pinv(L)
	
    # Compute velocity of the camera
	# vcam_vs = - lamb * np.squeeze(np.array(np.matmul(L_pseudo, error_vs.T)))
	vcam_vs = -lamb * np.linalg.pinv(L_pseudo).dot(error_vs)

	# Compute the velocity of the camera needed to reduce the error, applying a gain factor (lambda)
	# Make vcam 6x1 before transforming to robot frame
	# For option 1
	vcam_vs = np.array([0, 0, vcam_vs[0], 0, 0, vcam_vs[1]])

	# For option 2
	#vcam_vs = np.array([[0], vcam_vs[0], vcam_vs[1], [0], [0], [0]])


	# Define the transformation matrix from camera to robot frame using rotation and skew matrices
	R = np.matrix([[0, 0, -1.0], [-1.0, 0, 0], [0, 1.0, 0]])
	skew_pos = np.matrix([[0, 0, 0], [0, 0, -0.159], [0, 0.159, 0]])
	transform_cam_to_robot = np.block([
		[R, np.matmul(skew_pos, R)],
		[np.zeros((3, 3)), R]
	])
    # Transform velocity from camera frame to robot frame
	vrobot_vs = np.squeeze(np.array(transform(vcam_vs, transform_cam_to_robot)))

    # Make velocity screw	
	vel = Twist()
	vel.linear.x = vrobot_vs[0]
	vel.linear.y = vrobot_vs[1]
	vel.linear.z = vrobot_vs[2]
	vel.angular.x = vrobot_vs[3]
	vel.angular.y = vrobot_vs[4]
	vel.angular.z = vrobot_vs[5]

	forward_reverse = mapValueScalSat(vel.linear.x)
	lateral_left_right = mapValueScalSat(-vel.linear.y)
	ascend_descend = mapValueScalSat(vel.linear.z)
	roll_left_right = mapValueScalSat(vel.angular.x)
	pitch_left_right = mapValueScalSat(vel.angular.y)
	yaw_left_right = mapValueScalSat(-vel.angular.z)

	cal_PWM = [pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse, lateral_left_right]


	# Publish messages
	error_vs_msg = Float64MultiArray(data = error_vs)
	pub_error.publish(error_vs_msg)

	vcam_vs_msg = Float64MultiArray(data = vcam_vs)
	pub_vcam.publish(vcam_vs_msg)

	vrobot_vs_msg = Float64MultiArray(data = vrobot_vs)
	pub_vrov.publish(vrobot_vs_msg)
      
	cal_PWM_vs_msg = Float64MultiArray(data = cal_PWM)
	cal_PWM_pub.publish(cal_PWM_vs_msg)

	i_vs_msg = Float64MultiArray(data = i)
	i_pub.publish(i_vs_msg)
      
	'''------------------------Save---------------------------------------'''
	error_vs_sv.append(error_vs_msg)
	vcam_vs_sv.append(vcam_vs_msg)
	vrobot_vs_sv.append(vrobot_vs_msg)
	cal_PWM_vs_sv.append(cal_PWM_vs_msg)

	# Save data to CSV file
	depth_correction_data = pd.DataFrame({
        'error_vs_msg': vcam_vs_sv,
        'vcam_vs_sv': vcam_vs_sv,
        'vrobot_vs_sv': vrobot_vs_sv,
        'cal_PWM_vs_sv': cal_PWM_vs_sv
    })

    # Save DataFrame to CSV file
	depth_correction_data.to_csv('/home/ether/catkin_ws/src/autonomous_rov/script/depth_correction_data.csv', index=False)
	'''------------------------Save---------------------------------------'''
      
	# Send velocity screw to the low level control
	setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse, lateral_left_right)

def subscriber():
	rospy.Subscriber("joy", Joy, joyCallback)
	rospy.Subscriber("cmd_vel", Twist, velCallback)
	rospy.Subscriber("mavros/imu/data", Imu, OdoCallback)
	rospy.Subscriber("mavros/imu/water_pressure", FluidPressure, PressureCallback)
	#rospy.Subscriber("/dvl/data", DVL, DvlCallback)
	rospy.Subscriber("distance_sonar", Float64MultiArray, pingerCallback)
	rospy.Subscriber("desired_point",Float64MultiArray,desiredpointcallback, queue_size=1)
	rospy.Subscriber("tracked_point",Float64MultiArray,trackercallback, queue_size=1)
	
if __name__ == '__main__':
	#armDisarm(False)  # Not automatically disarmed at startup
	rospy.init_node('autonomous_MIR', anonymous=False)
	pub_msg_override = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size = 10, tcp_nodelay = True)
	pub_angle_degre = rospy.Publisher('angle_degree', Twist, queue_size = 10, tcp_nodelay = True)
	pub_depth = rospy.Publisher('depth/state', Float64, queue_size = 10, tcp_nodelay = True)
	#set_mode = [False, False, True]

	pub_angular_velocity = rospy.Publisher('angular_velocity', Twist, queue_size = 10, tcp_nodelay = True)
	pub_linear_velocity = rospy.Publisher('linear_velocity', Twist, queue_size = 10, tcp_nodelay = True)

	pub_error = rospy.Publisher("error",Float64MultiArray,queue_size=1,tcp_nodelay = True)
	pub_vcam = rospy.Publisher("Cam_velocity",Float64MultiArray,queue_size=1,tcp_nodelay = True)
	pub_vrov = rospy.Publisher("ROV_velocity",Float64MultiArray,queue_size=1,tcp_nodelay = True)
	cal_PWM_pub = rospy.Publisher("cal_PWM",Float64MultiArray,queue_size=1,tcp_nodelay = True)
	i_pub = rospy.Publisher("i_pub",Float64MultiArray,queue_size=1,tcp_nodelay = True)
	subscriber()

	rospy.spin()
