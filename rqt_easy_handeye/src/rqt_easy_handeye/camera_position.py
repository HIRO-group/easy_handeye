#!/usr/bin/env python
import rospy
import intera_interface
import rospkg
import os
import math
import random
import time


from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from tf.transformations import quaternion_from_euler
from trac_ik_python.trac_ik import IK


def main():

    rospy.init_node('init_sawyer', anonymous=True)
    limbs = intera_interface.Limb()
    quat = quaternion_from_euler(0, 0, 1.57)  # facing down, 1.57
    orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    rot = [orientation.x, orientation.y, orientation.z, orientation.w]

    limbs.move_to_neutral()


    print("Camera Scope Definition")
    print("Please move Sawyer's arm to the upper right corner of the camera frame. Raise Sawyer's arm towards the ceiling, as high as possible while the ArUco axes still register stably in the camera view")
    raw_input("Once in position, press enter to continue: ")

    upper_right = limbs.endpoint_pose()

    print("Upper right boundary:", upper_right)

    print

    print("Please move Sawyer's arm to the lower left corner of the camera frame. Lower Sawyer's arm towards the table, as low as possible while the ArUco axes still register stably in the camera view")
    raw_input("Press Enter to continue...")

    lower_left = limbs.endpoint_pose()

    print("Lower left boundary:", lower_left)

    # Formatting strings
    ur_string1 = str(upper_right["position"][0]) + ", " + str(upper_right["position"][1]) + ", " + str(upper_right["position"][2])
    ur_string2 = str(upper_right["orientation"][0]) + ", " + str(upper_right["orientation"][1]) + ", " + str(upper_right["orientation"][2]) + ", " + str(upper_right["orientation"][3])

    ll_string1 = str(lower_left["position"][0]) + ", " + str(lower_left["position"][1]) + ", " + str(lower_left["position"][2])
    ll_string2 = str(lower_left["orientation"][0]) + ", " + str(lower_left["orientation"][1]) + ", " + str(lower_left["orientation"][2]) + ", " + str(lower_left["orientation"][3])

    # Moving pose information to boundary.txt file
    text_file = open('boundary.csv','w')
    text_file.write(ur_string1 +"\n")
    text_file.write(ur_string2 +"\n")

    text_file.write(ll_string1 +"\n")
    text_file.write(ll_string2 +"\n")

    text_file.close()


    print
    print("These values have been passed to the boundary.csv file. Ready for automatic calibration.")

    print



if __name__ == '__main__':
    main()
