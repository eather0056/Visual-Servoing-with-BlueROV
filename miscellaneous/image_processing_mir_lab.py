#!/usr/bin/env python2
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import CompressedImage
import cv2
import camera_parameters as cam
import imutils
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn

# Data Publisher
Current_point_pub = rospy.Publisher('/vis/Current_point', Float64, queue_size=10)
Desiar_point_pub = rospy.Publisher('/vis/Desiar_point', Float64, queue_size=10)
pub_cmd_vel = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
pub_msg_override = rospy.Publisher('/mavros/rc/pub_msg_override', OverrideRCIn, queue_size=10)
pub_error_vs = rospy.Publisher('/vis/error_vs', Float64MultiArray, queue_size=10)
pub_vcam_vs = rospy.Publisher('/vis/vcam_vs', Twist, queue_size=10)
pub_vrobot_vs = rospy.Publisher('/vis/vrobot_vs', Twist, queue_size=10)

k = 0.5  # Control gain

# Assume desired_point_meter is a global variable to hold the desired point in meters
global desired_point_meter
desired_point_meter = np.array([0.0, 0.0])  # Initialize with the default desired point

# display point and text in an image
# param image : the image to overlay
# param pt : array that represents the 2D point in pixel 
# param r : red intensity
# param g : green intensity
# param b : blue intensity
# OPT param text : text to display near to the point, default empty
# OPT param scale : dot circle scale default 1
# OPT param offsetx : x offset of the text wrt the dot default 5
# OPT param offsety : y offset of the text wrt the dot default 5
def overlay_points(image, pt, r, g, b, text="", scale=1, offsetx=5, offsety=5):
    cv2.circle(image, (int(pt[0]), int(pt[1])), int(4*scale+1), (b,g,r),-1)
    position = (int(pt[0]) + offsetx, int(pt[1]) + offsety)
    cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX, scale, (b, g, r, 255), 1)

# Fonction that is called at every image frame
# by the image subscriber
# output the position in meter of 
# the point to track
def cameracallback(image_data):
   
	# get image data
	np_arr = np.fromstring(image_data.data, np.uint8)
	image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 

	# get image size
	image_height,image_width,image_channels = image_np.shape  
	print(image_height,image_width,image_channels)

	# display image data
	cv2.namedWindow("image")

	# broadcast center of the buoy
	# default coordinates in pixels
	desired_point = [image_width/2,image_height/2]

	#current_point = [50,50]
	#=================================#
	# TODO 1. track the buoy  :       #
	# insert your code here           #
	# to set the right current point  #

	hsv_img = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)

	# Previous Values
	# lower_range = (0, 50, 50) # lower range of red color in HSV
	# upper_range = (10, 255, 255) # upper range of red color in HSV

	lower_range = (120, 177, 190) # lower range of red color in HSV
	upper_range = (192, 233, 255) # upper range of red color in HSV



	mask = cv2.inRange(hsv_img, lower_range, upper_range)

	cv2.imshow('Mask', mask)
    
	non_zero_pixels = cv2.findNonZero(mask)
	
	if non_zero_pixels is not None and len(non_zero_pixels) > 0:

		color_image = cv2.bitwise_and(image_np, image_np, mask=mask)

		gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (5, 5), 0)
		thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY)[1]

		cv2.imshow('thresh', thresh)		


		cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		cnts = imutils.grab_contours(cnts)

		# find the biggest contour
		c = max(cnts, key = cv2.contourArea)

		# compute the center of the contour
		M = cv2.moments(c)
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])

		current_point = [cX, cY]

		#=================================#

		#display on the image
		overlay_points(image_np,current_point,0,255,0,'current tracked buoy')

		#display on the image
		overlay_points(image_np,desired_point,255,0,0,'desired point')

		#convert the point into meter
		current_point_meter = cam.convertOnePoint2meter (current_point)
		print('current points in meter',current_point_meter)    
		current_point_msg = Float64MultiArray(data = current_point_meter)
		pub_tracked_point.publish(current_point_msg)

		desired_point_meter = cam.convertOnePoint2meter (desired_point)
		print('desired point in meter',desired_point_meter)    
		desired_point_msg = Float64MultiArray(data = desired_point_meter)
		pub_desired_point.publish(desired_point_msg)
		trackercallback(current_point_msg)
		# current_point_meter_f = float(current_point_meter)
		# desired_point_meter_f = float(desired_point_meter)
		# current_point_meter_f = tuple(np.array(current_point_meter).astype(float64)) 
		# desired_point_meter_f = tuple(np.array(desired_point_meter).astype(float64))
		# Publish depth_wrt_startup and Correction_depth
		

		# point_msg = Point()
		# point_msg.x = current_point_meter[0]
		# point_msg.y = current_point_meter[1]
		# point_msg.z = 0  # If you're working in 2D space, set z to 0 or any default value

		# Current_point.publish(point_msg)

		# Current_point.publish(current_point_meter)
		# Desiar_point.publish(desired_point_meter)



		cv2.imshow("image", image_np)
		cv2.waitKey(2)


global desired_point_vs
pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

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

def trackercallback(data):
	global desired_point_meter

	# Assuming the message is a Float64MultiArray with data being a flat list [x, y]
	current_point_meter = np.array(data.data)  # Convert the list to a NumPy array for vectorized operations

	# Compute vector error
	error = current_point_meter - desired_point_meter
	computeVisualServoing(current_point_meter, desired_point_meter)

	# Apply control gain
	v = k * error

	vx, vy, vz, wx, wy, wz = 0, 0, 0, 0, 0, 0

	vrobot = [vx,vy,vz,wx,wy,wz]

	vel = Twist()
	
	vel.linear.x = vrobot[0]
	vel.linear.y = vrobot[1]
	vel.linear.z = vrobot[2]
	vel.angular.x = vrobot[3]
	vel.angular.y = vrobot[4]
	vel.angular.z = vrobot[5]

	pub_cmd_vel.publish(vel)

	forward_reverse = mapValueScalSat(vel.linear.x)
	lateral_left_right = mapValueScalSat(-vel.linear.y)
	ascend_descend = mapValueScalSat(vel.linear.z)
	roll_left_right = mapValueScalSat(vel.angular.x)
	pitch_up_down = mapValueScalSat(vel.angular.y)
	yaw_left_right = mapValueScalSat(-vel.angular.z)

	setOverrideRCIN(pitch_up_down, roll_left_right, ascend_descend, yaw_left_right, forward_reverse, lateral_left_right)

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

def computeVisualServoing(current_point, desired_point):
	# Compute visual servoing error
	error_vs = np.array(desired_point) - np.array(current_point)

	# Compute interaction matrices for current, desired, and mixed scenarios
	currentL = interactionMatrixFeaturePoint2D(current_point[0],current_point[1], Z=1)
	desiredL = interactionMatrixFeaturePoint2D(desired_point[0],desired_point[1], Z=1)
	mixteL = (currentL + desiredL) / 2

	# Placeholder for computing camera velocity based on the visual servoing error
	# This computation will depend on your specific visual servoing approach.
	vcam_vs = np.linalg.inv(mixteL).dot(error_vs)  # Simplified example

	# Placeholder transformation matrix from camera to robot frame
	homogeneous_transform_from_cam_to_robot = np.eye(4)  # Identity matrix as a placeholder

	# Transform camera velocity to robot velocity
	vrobot_vs = transform(vcam_vs, homogeneous_transform_from_cam_to_robot)
	print(error_vs.tolist())

	# Publishing results
	pub_error_vs.publish(Float64MultiArray(data=error_vs.tolist()))
	pub_vcam_vs.publish(Twist(linear=vcam_vs[:3], angular=vcam_vs[3:]))
	pub_vrobot_vs.publish(Twist(linear=vrobot_vs[:3], angular=vrobot_vs[3:]))

	return error_vs, vcam_vs, vrobot_vs

def transform(vcam, transform_matrix):
    """
    Transform camera velocities to robot velocities using a given transformation matrix.
    This is a placeholder function. You should replace it with your actual transformation logic.
    """
    # This example simply applies the transformation matrix to the camera velocities.
    vrobot = np.dot(transform_matrix, vcam)
    return vrobot

# the interactionMatrix for point coordinates
def interactionMatrixFeaturePoint2D(x,y,Z=1):
	Lx = np.array([ -1 / Z, 0, x / Z, x * y,-(1 + x * x), y])
	Ly = np.array([0, -1 / Z, y / Z, 1 + y * y, -x * y, -x ])
	L = np.stack((Lx.T,Ly.T),axis=0)
	return L

# subscribers
def subscribers():
    # default image topic
	image_topic_name = "/br2/raspicam_node/image/compressed"

	## get image topic name from the launchfile/prompt parameters
		
	if rospy.has_param('~cam_name'):
		image_topic_name =rospy.get_param('~cam_name')
		print ("Camera Name = ", image_topic_name, ".")
	else:
		rospy.logwarn('no camera parameter given; using the default value %s' %image_topic_name)
	print(image_topic_name)
	rospy.Subscriber(image_topic_name, CompressedImage, cameracallback,  queue_size = 1)
    


# publishers
def publishers():
	global pub_tracked_point
	global pub_desired_point
	pub_tracked_point = rospy.Publisher("tracked_point",Float64MultiArray,queue_size=1,tcp_nodelay = True)
	pub_desired_point = rospy.Publisher("desired_point",Float64MultiArray,queue_size=1,tcp_nodelay = True)

if __name__ == '__main__':
    
	rospy.init_node('image_processing_mir', anonymous=False)  
	print('image processing launched')
	publishers()
	subscribers()
	rospy.spin()  # Execute  in loop


