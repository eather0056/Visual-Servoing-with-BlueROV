#!/usr/bin/env python2
# Import necessary libraries
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import CompressedImage
import cv2
import camera_parameters as cam  # Custom module for camera parameters
import imutils  # Utility library for image processing simplification

# Function to overlay points and optional text on an image
def overlay_points(image, pt, r, g, b, text="", scale=1, offsetx=5, offsety=5):
    # Draw a circle at the given point with specified color and scale
    cv2.circle(image, (int(pt[0]), int(pt[1])), int(4*scale+1), (b, g, r), -1)
    # Calculate text position based on offset
    position = (int(pt[0])+offsetx, int(pt[1])+offsety)
    # Put text on the image if provided
    cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX, scale, (b, g, r, 255), 1)

# Callback function for image subscriber; processes each frame received
def cameracallback(image_data):
    # Decode the compressed image
    np_arr = np.fromstring(image_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # Display the dimensions of the image for debugging
    image_height, image_width, image_channels = image_np.shape  
    #print(image_height, image_width, image_channels)

    # Prepare window to display image (for debugging)
    cv2.namedWindow("image")

    # Set the desired point at the center of the image
    desired_point = [image_width/2, image_height/2]

    # Convert BGR image to HSV color space
    hsv_img = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
    # Define the HSV range for the target color
    lower_range = (120, 177, 190)  # Adjust these values to target color
    upper_range = (192, 233, 255)

    # Create a mask that isolates the target color in the image
    mask = cv2.inRange(hsv_img, lower_range, upper_range)

    # Display the mask for debugging
    cv2.imshow('Mask', mask)
    
    # Find non-zero (target color) pixel locations in the mask
    non_zero_pixels = cv2.findNonZero(mask)
    if non_zero_pixels is not None and len(non_zero_pixels) > 0:
        # Apply mask to the original image to isolate the target color
        color_image = cv2.bitwise_and(image_np, image_np, mask=mask)
        # Convert to grayscale for contour detection
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        # Blur and threshold the image to prepare for contour detection
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY)[1]

        # Find contours in the thresholded image
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        # The largest contour to be the target buoy
        c = max(cnts, key=cv2.contourArea)
        # Calculate the centroid of the contour
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        current_point = [cX, cY]

        # Overlay the current and desired points on the image for visualization
        overlay_points(image_np, current_point, 0, 255, 0, 'tracked buoy')
        overlay_points(image_np, desired_point, 255, 0, 0, 'desired point')

        # Convert pixel positions to meters using camera parameters
        current_point_meter = cam.convertOnePoint2meter(current_point)
        desired_point_meter = cam.convertOnePoint2meter(desired_point)

        # Publish the current and desired points in meters to ROS topics
        pub_tracked_point.publish(Float64MultiArray(data=current_point_meter))
        pub_desired_point.publish(Float64MultiArray(data=desired_point_meter))

        # Display the processed image with overlays
        cv2.imshow("image", image_np)
        cv2.waitKey(2)

# Function to setup subscribers
def subscribers():
    # Set default image topic name
    image_topic_name = "/br2/raspicam_node/image/compressed"
    # Override default topic name if provided as a parameter
    if rospy.has_param('~cam_name'):
        image_topic_name = rospy.get_param('~cam_name')
    else:
        rospy.logwarn('no camera parameter given; using the default value %s' % image_topic_name)
    rospy.Subscriber(image_topic_name, CompressedImage, cameracallback,  queue_size=1)

# Function to setup publishers
def publishers():
    global pub_tracked_point, pub_desired_point
    pub_tracked_point = rospy.Publisher("tracked_point", Float64MultiArray, queue_size=1, tcp_nodelay=True)
    pub_desired_point = rospy.Publisher("desired_point", Float64MultiArray, queue_size=1, tcp_nodelay=True)

if __name__ == '__main__':
    rospy.init_node('image_processing_mir', anonymous=False)
    print('image processing launched')
    publishers()
    subscribers()
    rospy.spin()  # Keep the node running
