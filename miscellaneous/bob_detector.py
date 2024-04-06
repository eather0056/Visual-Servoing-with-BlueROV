clc#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray

def find_bob(image):
    # Convert the image from BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_color = np.array([0, 100, 100])  # Set lower bound of the object's color
    upper_color = np.array([10, 255, 255])  # Set upper bound of the object's color

    # Create a mask for the color
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If contours are found, assume the largest is the object
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return cx, cy
    return None, None

def image_callback(msg):
    # Convert compressed ROS image to OpenCV image
    np_arr = np.fromstring(msg.data, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # Object Detection
    cx, cy = find_bob(image)
    if cx is not None and cy is not None:
        # Overlay detected point
        cv2.circle(image, (cx, cy), 10, (0, 255, 0), -1)

        rospy.loginfo('Detected object at pixel coordinates: ({}, {})'.format(cx, cy))

    else:
        rospy.loginfo('Object not found')

    # Display the image
    cv2.imshow("Image window", image)
    cv2.waitKey(2)

    # Display the image
    cv2.imshow("Image window", image)
    cv2.waitKey(2)

rospy.init_node('bob_detector')
image_topic = "/br2/raspicam_node/image/compressed"  # Topic to subscribe
pub = rospy.Publisher("detected_point", Float64MultiArray, queue_size=10)

rospy.Subscriber(image_topic, CompressedImage, image_callback)
rospy.spin()
cv2.destroyAllWindows()

