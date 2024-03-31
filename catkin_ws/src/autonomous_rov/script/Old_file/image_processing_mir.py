#!/usr/bin/env python3
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
def overlay_points(image, pt, r, g, b, text="", scale=1, offsetx=5, offsety=5):
    cv2.circle(image, (int(pt[0]), int(pt[1])), int(4 * scale + 1), (b, g, r), -1)
    position = (int(pt[0]) + offsetx, int(pt[1]) + offsety)
    cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX, scale, (b, g, r, 255), 1)

# Function that is called at every image frame by the image subscriber
def cameracallback(image_data):
    # Get image data
    np_arr = np.fromstring(image_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # Get image size
    image_height, image_width, image_channels = image_np.shape
    print(image_height, image_width, image_channels)

    # Display image data
    cv2.namedWindow("image")

    # Broadcast center of the buoy
    # Default coordinates in pixels
    desired_point = [image_width / 2, image_height / 2]
    current_point = [50, 50]

    # =================================#
    # TODO 1. Track the Buoy:          #
    # Insert your code here to track   #
    # the buoy and set the right       #
    # current_point                     #

    # Convert the image from BGR to HSV
    hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the red color 
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    # Create a mask for the red color
    mask = cv2.inRange(hsv, lower_red, upper_red)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Check if any contour is found
    if contours:
        # Get the largest contour (assumed to be the red buoy)
        largest_contour = max(contours, key=cv2.contourArea)

        # Calculate the center of the contour
        M = cv2.moments(largest_contour)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        current_point = [cx, cy]
    else:
        # If no contour is found, keep the current point the same (no tracking)
        pass

    # =================================#

    # Display on the image
    overlay_points(image_np, current_point, 0, 255, 0, 'current tracked buoy')
    overlay_points(image_np, desired_point, 255, 0, 0, 'desired point')

    # Convert the point into meters
    current_point_meter = cam.convertOnePoint2meter(current_point)
    print('current points in meter', current_point_meter)
    current_point_msg = Float64MultiArray(data=current_point_meter)
    pub_tracked_point.publish(current_point_msg)

    desired_point_meter = cam.convertOnePoint2meter(desired_point)
    print('desired point in meter', desired_point_meter)
    desired_point_msg = Float64MultiArray(data=desired_point_meter)
    pub_desired_point.publish(desired_point_msg)

    cv2.imshow("image", image_np)
    cv2.waitKey(2)

# Subscribers
def subscribers():
    # Default image topic
    #image_topic_name = "usb_cam/image/compressed"
    image_topic_name = "/br2/usb_cam/image_raw/compressed"
    #image_topic_name = "/br4/raspicam_node/image/compressed"

    # Get image topic name from the launch file/prompt parameters
    if rospy.has_param('~cam_name'):
        image_topic_name = rospy.get_param('~cam_name')
        print("Camera Name =", image_topic_name)
    else:
        rospy.logwarn('no camera parameter given; using the default value %s' % image_topic_name)

    rospy.Subscriber(image_topic_name, CompressedImage, cameracallback, queue_size=1)

# Publishers
def publishers():
    global pub_tracked_point, pub_desired_point
    pub_tracked_point = rospy.Publisher("tracked_point", Float64MultiArray, queue_size=1, tcp_nodelay=True)
    pub_desired_point = rospy.Publisher("desired_point", Float64MultiArray, queue_size=1, tcp_nodelay=True)

if __name__ == '__main__':
    rospy.init_node('image_processing_mir', anonymous=False)
    print('image processing launched')
    publishers()
    subscribers()
    rospy.spin()  # Execute in a loop
