#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import CompressedImage
import cv2
import camera_parameters as cam



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
def overlay_points(image,pt,r,g,b,text="",scale =1,offsetx=5, offsety=5):
	cv2.circle(image,(int(pt[0]),int(pt[1])),int(4*scale+1), (b,g,r),-1)
	position = (int(pt[0])+offsetx,int(pt[1])+offsety)
	cv2.putText(image,text,position,cv2.FONT_HERSHEY_SIMPLEX, scale,(b, g, r, 255),1)


# Fonction that is called at every image frame
# by the image subscriber
# output the position in meter of 
# the point to track
get_hsv = False

def click_detect(event,x, y, flags, param):
	global get_hsv,mouseX,mouseY
	if event == cv2.EVENT_LBUTTONDOWN:
		get_hsv =True
		mouseX, mouseY =x,y
   	 
def cameracallback(image_data):
	global get_hsv,mouseX,mouseY
	# get image data
	np_arr = np.fromstring(image_data.data, np.uint8)
	image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
	if get_hsv==True :
		hsv_value = hsv[mouseY, mouseX]
		print("HSV Value at ({}, {}): {}".format(mouseX, mouseY, hsv_value))    
		get_hsv=False


	# get image size
	image_height,image_width,image_channels = image_np.shape  
	#print(image_height,image_width,image_channels)

	# display image data
	cv2.namedWindow("image")

	cv2.setMouseCallback("image", click_detect)
	# broadcast center of the buoy
	# default coordinates in pixels
	desired_point = [image_width/2,image_height/2]

	current_point = [image_width/2,image_height/2]
	#=================================#
	# TODO 1. track the buoy  :       #
	# insert your code here           #
	# to set the right current point  #
	hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)

	lower_bound = np.array([150, 100, 100])
	upper_bound = np.array([180, 255, 255])

	mask = cv2.inRange(hsv, lower_bound, upper_bound)
	cv2.imshow('Mask', mask)

	non_zero_pixels = cv2.findNonZero(mask)

	if non_zero_pixels is not None and len(non_zero_pixels) > 0:
		mean_point = np.mean(non_zero_pixels, axis=0, dtype=np.int)
		cx, cy = mean_point[0]

		# Draw a circle at the center
		cv2.circle(image_np, (cx, cy), 5, (0, 255, 0), -1)

		#print("Center Pixel of Blob: ({}, {})".format(cx, cy))
		current_point = [cx, cy]

	#=================================#

	#display on the image
	overlay_points(image_np,current_point,0,255,0,'current tracked buoy')

	#display on the image
	overlay_points(image_np,desired_point,255,0,0,'desired point')

	#convert the point into meter
	current_point_meter = cam.convertOnePoint2meter (current_point)
	#print('current points in meter',current_point_meter)    
	current_point_msg = Float64MultiArray(data = current_point_meter)
	pub_tracked_point.publish(current_point_msg)

	desired_point_meter = cam.convertOnePoint2meter (desired_point)
	#print('desired point in meter',desired_point_meter)    
	desired_point_msg = Float64MultiArray(data = desired_point_meter)
	pub_desired_point.publish(desired_point_msg)


	cv2.imshow("image", image_np)
	cv2.waitKey(2)


# subscribers
def subscribers():
	# default image topic
	#image_topic_name = "usb_cam/image/compressed";
	image_topic_name = "/br2/raspicam_node/image/compressed";

	# get image topic name from the launchfile/prompt parameters
	if rospy.has_param('~cam_name'):
		image_topic_name =rospy.get_param('~cam_name')
		print ("Camera Name = ", image_topic_name, ".")
	else:
		rospy.logwarn('no camera parameter given; using the default value %s' %image_topic_name)

	rospy.Subscriber(image_topic_name, CompressedImage, cameracallback,  queue_size = 1)



# publishers
def publishers():
	global pub_tracked_point, pub_desired_point
	pub_tracked_point = rospy.Publisher("tracked_point",Float64MultiArray,queue_size=1,tcp_nodelay = True)
	pub_desired_point = rospy.Publisher("desired_point",Float64MultiArray,queue_size=1,tcp_nodelay = True)

if __name__ == '__main__':

	rospy.init_node('image_processing_mir', anonymous=False)  
	print('image processing launched')
	publishers()
	subscribers()
	rospy.spin()  # Execute  in loop


