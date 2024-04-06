#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import Joy
import pygame
import sys

# Initialize pygame and the joystick-like message
pygame.init()
screen = pygame.display.set_mode((100, 100))
pygame.display.set_caption('ROS Keyboard Teleop')

def send_joy_message(btn_arm, btn_disarm, btn_manual_mode, btn_automatic_mode, btn_corrected_mode):
    pub = rospy.Publisher('joy', Joy, queue_size=1)
    rospy.init_node('keyboard_joy')
    rate = rospy.Rate(10)  # 10hz

    joy_msg = Joy()
    joy_msg.buttons = [0]*12  # Initialize all buttons to 0

    # Update buttons based on keyboard input
    joy_msg.buttons[7] = btn_arm
    joy_msg.buttons[6] = btn_disarm
    joy_msg.buttons[3] = btn_manual_mode
    joy_msg.buttons[2] = btn_automatic_mode
    joy_msg.buttons[0] = btn_corrected_mode

    while not rospy.is_shutdown():
        pub.publish(joy_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_a:
                        send_joy_message(1, 0, 0, 0, 0)
                    elif event.key == pygame.K_b:
                        send_joy_message(0, 1, 0, 0, 0)
                    # Map more keys as needed
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
    except rospy.ROSInterruptException:
        pass
