#!/usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import CompressedImage
import cv2
import camera_parameters as cam

get_hsv = False

def click_detect(event, x, y, flags, param):
    global get_hsv, mouseX, mouseY
    if event == cv2.EVENT_LBUTTONDOWN:
        get_hsv = True
        mouseX, mouseY = x, y

def overlay_points(image, pt, r, g, b, text="", scale=1, offsetx=5, offsety=5):
    """Display point and text in an image"""
    cv2.circle(image, (int(pt[0]), int(pt[1])), int(4 * scale + 1), (b, g, r), -1)
    position = (int(pt[0]) + offsetx, int(pt[1]) + offsety)
    cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX, scale, (b, g, r, 255), 1)

def cameracallback(image_data):
    global get_hsv, mouseX, mouseY

    # Get image data
    np_arr = np.fromstring(image_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # Get image size
    image_height, image_width, image_channels = image_np.shape
    print(image_height, image_width, image_channels)

    cv2.namedWindow("image")
    cv2.namedWindow("mask")

    desired_point = [image_width / 2, image_height / 2]
    current_point = [image_width / 2, image_height / 2]

    hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
    cv2.setMouseCallback("image", click_detect)

    lower_red = np.array([150, 100, 100])
    upper_red = np.array([180, 255, 255])

    mask = cv2.inRange(hsv, lower_red, upper_red)

    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx, cy = 0, 0
        current_point = [cx, cy]
    
    non_zero_pixels = cv2.findNonZero(mask)
    if non_zero_pixels is not None and len(non_zero_pixels) > 0:
        mean_point = np.mean(non_zero_pixels, axis=0, dtype=np.int)
        cx, cy = mean_point[0]
        cv2.circle(image_np, (cx, cy), 5, (0, 255, 0), -1)

    overlay_points(image_np, current_point, 0, 255, 0, 'current tracked buoy')
    overlay_points(image_np, desired_point, 255, 0, 0, 'desired point')

    current_point_meter = cam.convertOnePoint2meter(current_point)
    current_point_msg = Float64MultiArray(data=current_point_meter)

    desired_point_meter = cam.convertOnePoint2meter(desired_point)
    desired_point_msg = Float64MultiArray(data=desired_point_meter)

    cv2.imshow("image", image_np)
    cv2.imshow("mask", mask)
    cv2.waitKey(2)

def subscribers():
    image_topic_name = "/br2/raspicam_node/image/compressed"

    if rospy.has_param('~cam_name'):
        image_topic_name = rospy.get_param('~cam_name')
        print("Camera Name =", image_topic_name)
    else:
        rospy.logwarn('no camera parameter given; using the default value %s' % image_topic_name)

    rospy.Subscriber(image_topic_name, CompressedImage, cameracallback, queue_size=1)

def publishers():
    global pub_tracked_point, pub_desired_point
    pub_tracked_point = rospy.Publisher("tracked_point", Float64MultiArray, queue_size=1, tcp_nodelay=True)
    pub_desired_point = rospy.Publisher("desired_point", Float64MultiArray, queue_size=1, tcp_nodelay=True)

if __name__ == '__main__':
    rospy.init_node('image_processing_mir', anonymous=False)
    print('image processing launched')
    publishers()
    subscribers()
    rospy.spin()
