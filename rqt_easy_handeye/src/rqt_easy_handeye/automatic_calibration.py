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
from sawyer_pykdl import sawyer_kinematics
from trac_ik_python.trac_ik import IK

import automatic_subscriber as subscriber

from easy_handeye.handeye_client import HandeyeClient
#from easy_handeye.handeye_calibrator import HandeyeCalibrator
pose_msgs = []
client = HandeyeClient()


def main():

    rospy.init_node('init_sawyer', anonymous=True)
    limbs = intera_interface.Limb()
    quat = quaternion_from_euler(0, 0, 1.57)  # facing down, 1.57
    orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    rot = [orientation.x, orientation.y, orientation.z, orientation.w]

    #calibrator = HandeyeCalibrator()
    
    global client
    limbs.move_to_neutral()


    
    # Removing intermediate starting poses with implementation of trac_ik instead of sawyer_kinematics
    # Intermediate starting poses
    #Pose 1
    #move_to_pose(limbs,[[0.518465588433949, -0.33558656378929175, 0.3512422480288401], [0.8475418601147259, 0.5292733980405926, -0.03910011369276156, 0.0036941289972406304]]) 

    #Pose 2
    #move_to_pose(limbs,[[0.22504996570245858, -0.6491949340661637, 0.5268105858083507], [0.8813086619557776, 0.2153199635246504, 0.1361836138671823, 0.39797786242897304]])

    #Pose 3
    #move_to_pose(limbs,[[0.1570186683617708, -0.7789679314951172, 0.6364141008459059], [0.6264309202569877, 0.1975130277440453, -0.17395048132946492, 0.7336989410259571]])
    
    #Pose 4
    #move_to_pose(limbs,[[0.4001265561768291, -0.6906366099358089, 0.7033218178851106], [0.22016963569645454, 0.2142998580427028, -0.11806222221637198, 0.9442786739334422]])

    


    # Starting pose: leaves Sawyer arm sitting centered within the camera frame, with the aruco tracker facing up and visible to camera
    move_to_pose(limbs,[[0.07093494823198944, -0.5598168972845347, 0.5455573236884597], [0.02295719804731124, 0.047233793059366, -0.02728651852509384, 0.9982471546454925]])


    # 3-D Grid Calculation
    # Upper Right Corner Pose
    pose_up_right = [[0.4328961149971866, -0.2791028804775756, 0.6830602085852143], [-0.03427450377263718, 0.0033527064940575605, 0.036570022312265485, 0.998737528692291]]

    # Lower Left Corner Pose
    pose_low_left = [[-0.3457073085378527, -0.7666996654555039, 0.37493469217616177], [-0.014338377202480697, -0.021029440780254635, -0.7005074551266381, 0.7131910535555468]]
    
    # Start and end pose variables
    # pose_low_left = (x0, y0, z0) = Start posiiton
    x0 = pose_low_left[0][0]
    y0 = pose_low_left[0][1]
    z0 = pose_low_left[0][2]

    # pose_up_right = (x1, y1, z1) = End position
    x1 = pose_up_right[0][0]
    x2 = pose_up_right[0][1]
    x3 = pose_up_right[0][2]



    # Calculations for length of the grid dimensions
    x_length = abs(pose_up_right[0][0] - pose_low_left[0][0])
    y_length = abs(pose_up_right[0][1] - pose_low_left[0][1])
    z_length = abs(pose_up_right[0][2] - pose_low_left[0][2])



    # Grid partitions - the length of x in this example is greater than those of y and z, so it is partitioned into 4 parts instead of 3 parts like y and z
    x_part = x_length/4
    y_part = y_length/3
    z_part = z_length/3


    # Partition Ranges
    x_part_ranges = [[x0, (x0 + x_part)], [(x0 + x_part), (x0 + 2*x_part)], [(x0 + 2*x_part), (x0 + 3*x_part)], [(x0 + 3*x_part), (x0 + 4*x_part)]]
    y_part_ranges = [[y0, (y0 + y_part)], [(y0 + y_part), (y0 + 2*y_part)], [(y0 + 2*y_part), (y0 + 3*y_part)]]
    z_part_ranges = [[z0, (z0 + z_part)], [(z0 + z_part), (z0 + 2*z_part)], [(z0 + 2*z_part), (z0 + 3*z_part)]]

    # Moving in the Grid

    # Start at lower left corner (x0,y0,z0)
    move_to_pose(limbs, pose_low_left)

    #up_rotation = [0.02295719804731124, 0.047233793059366, -0.02728651852509384, 0.9982471546454925]
    
    # Loops through the ranges of each dimension, moving along the x-axis, then the y-axis, and finally the z-axis
    # Random choices for the x, y, and z coordinates within their ranges for that "cube" of the grid
    # Not sure how small we want the step - with step = 0.025, there are 8 options within the x ranges, 8 options within the y ranges, and 4 options within the z ranges


    #test_x = random.uniform(x_part_ranges[0][0], x_part_ranges[0][1])
    #test_y = random.uniform(y_part_ranges[0][0], y_part_ranges[0][1])
    #test_z = random.uniform(z_part_ranges[0][0], z_part_ranges[0][1])

    #move_to_pose(limbs, [[test_x, test_y, test_z], up_rotation])

    

    z_range = z_part_ranges[0]
    y_range = y_part_ranges[0]
    for x_range in x_part_ranges:
        x_coor = random.uniform(x_range[0], x_range[1])
        y_coor = random.uniform(y_range[0], y_range[1])
        z_coor = random.uniform(z_range[0], z_range[1])
        quatern = random_quat()
        move_to_pose(limbs,[[x_coor, y_coor, z_coor], quatern])
        
        stable = check_aruco()

        if stable:
            print("Aruco is stable. Taking sample.")
            take_a_sample()
        else:
            print("Aruco is not stable. Moving on to next position")


    print('\n')
    z_range = z_part_ranges[0]
    y_range = y_part_ranges[1]
    for x_range in reversed(x_part_ranges):
        x_coor = random.uniform(x_range[0], x_range[1])
        y_coor = random.uniform(y_range[0], y_range[1])
        z_coor = random.uniform(z_range[0], z_range[1])
        quatern = random_quat()
        move_to_pose(limbs,[[x_coor, y_coor, z_coor], quatern])
 
        stable = check_aruco()

        if stable:
            print("Aruco is stable. Taking sample.")
            take_a_sample()
        else:
            print("Aruco is not stable. Moving on to next position")
            
    print('\n')

    # Skipping y[2] range for z[0] level due to frequency of Sawyer's arm hitting its base
    #z_range = z_part_ranges[0]
    #y_range = y_part_ranges[2]
    #for x_range in x_part_ranges:
        #x_coor = random.uniform(x_range[0], x_range[1])
        #y_coor = random.uniform(y_range[0], y_range[1])
        #z_coor = random.uniform(z_range[0], z_range[1])
        #move_to_pose(limbs,[[x_coor, y_coor, z_coor], up_rotation])


   #Some weird pauses when y = 2 (closest to base of Sawyer) - possibly hard to compute path when Sawyer needs to plan around its body?

   
    print('\n')
   
   # Z level 1
    z_range = z_part_ranges[1]
    y_range = y_part_ranges[0]
    for x_range in reversed(x_part_ranges):
        x_coor = random.uniform(x_range[0], x_range[1])
        y_coor = random.uniform(y_range[0], y_range[1])
        z_coor = random.uniform(z_range[0], z_range[1])
        quatern = random_quat()
        move_to_pose(limbs,[[x_coor, y_coor, z_coor], quatern])

        stable = check_aruco()

        if stable:
            print("Aruco is stable. Taking sample.")
            take_a_sample()
        else:
            print("Aruco is not stable. Moving on to next position")

    print('\n')

    z_range = z_part_ranges[1]
    y_range = y_part_ranges[1]
    for x_range in x_part_ranges:
        x_coor = random.uniform(x_range[0], x_range[1])
        y_coor = random.uniform(y_range[0], y_range[1])
        z_coor = random.uniform(z_range[0], z_range[1])
        quatern = random_quat()
        move_to_pose(limbs,[[x_coor, y_coor, z_coor], quatern])

        stable = check_aruco()
        
        if stable:
            print("Aruco is stable. Taking sample.")
            take_a_sample()
        else:
            print("Aruco is not stable. Moving on to next position")
    
    print('\n')
    

    # Skipping y[2] range for z[1] level due to frequency of Sawyer's arm hitting its base
    #z_range = z_part_ranges[1]
    #y_range = y_part_ranges[2]
    #for x_range in reversed(x_part_ranges):
        #x_coor = random.uniform(x_range[0], x_range[1])
        #y_coor = random.uniform(y_range[0], y_range[1])
        #z_coor = random.uniform(z_range[0], z_range[1])
        #move_to_pose(limbs,[[x_coor, y_coor, z_coor], up_rotation])


    print('\n')

    # Z Level 2
    z_range = z_part_ranges[2]
    y_range = y_part_ranges[0]
    for x_range in x_part_ranges:
        x_coor = random.uniform(x_range[0], x_range[1])
        y_coor = random.uniform(y_range[0], y_range[1])
        z_coor = random.uniform(z_range[0], z_range[1])
        quatern = random_quat()
        move_to_pose(limbs,[[x_coor, y_coor, z_coor], quatern])

        stable = check_aruco()

        if stable:
            print("Aruco is stable. Taking sample.")
            take_a_sample()
        else:
            print("Aruco is not stable. Moving on to next position")

    print('\n')

    z_range = z_part_ranges[2]
    y_range = y_part_ranges[1]
    for x_range in reversed(x_part_ranges):
        x_coor = random.uniform(x_range[0], x_range[1])
        y_coor = random.uniform(y_range[0], y_range[1])
        z_coor = random.uniform(z_range[0], z_range[1])
        quatern = random_quat()
        move_to_pose(limbs,[[x_coor, y_coor, z_coor], quatern])

        stable = check_aruco()

        if stable:
            print("Aruco is stable. Taking sample.")
            take_a_sample()
        else:
            print("Aruco is not stable. Moving on to next position")

    print('\n')



    # Though this level of z yields less impacts when using the y[2] range, the movements are punctuated with ~5-15 second pauses
    z_range = z_part_ranges[2]
    y_range = y_part_ranges[2]
    for x_range in x_part_ranges:
        x_coor = random.uniform(x_range[0], x_range[1])
        y_coor = random.uniform(y_range[0], y_range[1])
        z_coor = random.uniform(z_range[0], z_range[1])
        quatern = random_quat()
        move_to_pose(limbs,[[x_coor, y_coor, z_coor], quatern])

        stable = check_aruco()

        if stable:
            print("Aruco is stable. Taking sample.")
            take_a_sample()
        else:
            print("Aruco is not stable. Moving on to next position")


    # Calculating Calibration
    initial_result = client.compute_calibration()

    print("Calibration following:")

    if initial_result.valid:
        print("Initial Calibration Result: " + str(initial_result))
    else:
        print('The calibration could not be computed. Please run the calibration script again')
        return
    

    # Temporarily creating a y_part_ranges list for random calibration confirmation to remove the last element of y_part_ranges that causes issues with Sawyer's arm hitting the body
    y_part_ranges_rand = y_part_ranges[0:1]
    # Taking a 3 more samples to confirm validity of calibration
    for i in range(3):
        x_range = random.choice(x_part_ranges)
        y_range = random.choice(y_part_ranges_rand)
        z_range = random.choice(z_part_ranges)
        
        x_coor = random.uniform(x_range[0], x_range[1])
        y_coor = random.uniform(y_range[0], y_range[1])
        z_coor = random.uniform(z_range[0], z_range[1])
        quatern = random_quat()
        move_to_pose(limbs,[[x_coor, y_coor, z_coor], quatern])

        stable = check_aruco()

        if stable:
            print("Aruco is stable. Taking sample.")
            take_a_sample()
        else:
            print("Aruco is not stable. Moving on to next position")

        print("Successful check" + str(i+1) + ": (" + str(x_coor) +", " + str(y_coor) + ", " + str(z_coor) + ")")

    # Calculating check calibration 
    check_result = client.compute_calibration()
    if check_result.valid:
        print("Check Calibration Result: " + str(check_result))
    else:
        print("Check calibration could not be computer. Please run the calibration script again.")
        return

    # Determining calibration stability


def get_pose(




def move_to_pose(limb, pose):
    #time.sleep(2.0)
    pos, rot = pose
 
    #trac_ik kinematics intializing
    ik_solver = IK("base","stp_021808TP00080_tip")

    current_joint_angles = limb.joint_angles()
    sorted_angles = sorted(current_joint_angles.items(), key = lambda kv:(kv[0], kv[1]))

    seed_state = [0] * ik_solver.number_of_joints
    for i in range(len(sorted_angles)):
        seed_state[i] = sorted_angles[i][1]
    
    kin = ik_solver.get_ik(seed_state, pos[0], pos[1] , pos[2], rot[0], rot[1], rot[2], rot[3])
    
    if kin == None:
        return None
    
    go_to_pose = dict(zip(limb.joint_names(), kin))
    limb.move_to_joint_positions(go_to_pose)

    #kin = sawyer_kinematics('right')
    #joint_pos = kin.inverse_kinematics(pos, rot)
    
    # Prints current location of Sawyer's endopoint (gripper)
    #print(limb.endpoint_pose())






# Checks if the ArUco marker is stable - returns a boolean True if it is, and a boolean False if not
def check_aruco():
    #Not sure this needs to be global anymore
    global pose_msgs
    
    stable = bool(0)
    
    # Take 30 samples
    for i in range(30):
        try:
            message = rospy.wait_for_message("aruco_tracker/pose", PoseStamped, timeout=3)
            pose_msgs.append(message)
        except:
          #  print("Aruco is not stable. Moving on to next position")
            return bool(0)
     

    # Creates a timestamps array for the pose and populates it with the time stamps of the pose messages 
    timestamps = []
    for signal in pose_msgs:
        s_timestamp = signal.header.stamp.secs
        timestamps.append(s_timestamp)

    if timestamps: 
        # Calcuates the time difference between the first and last time stamp
        time_delta = timestamps[len(timestamps)-1] - timestamps[0]
        #print("Difference between timestamps: " + str(time_delta) + " seconds")

        # If time delay was under 3 seconds for 30 samples, return true - ArUco tracker is stable and calibrate-able
        if time_delta <= 3:
            stable = bool(1)
        else:
            stable = bool(0)

    # Clear the pose_msgs global variable
    pose_msgs = []
    

    return stable

def random_quat():
    # Roll, Pitch and Yaw Randomization in Euler
    roll_range = [-0.38, 0.2]
    pitch_range = [0.07, 0.1]
    yaw_range = [-math.pi, math.pi]
    
    roll = random.uniform(roll_range[0], roll_range[1])
    pitch = random.uniform(pitch_range[0], pitch_range[1])
    yaw = random.uniform(yaw_range[0], yaw_range[1])


    quat = quaternion_from_euler(roll, pitch, yaw)  # facing down, 1.57
    orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    rot = [orientation.x, orientation.y, orientation.z, orientation.w]

    return rot



def take_a_sample():
    global client
    sample_list = client.take_sample()
    #for i in range(len(sample_list.hand_world_samples.transforms)):
        #print(str(i+1) + ") \n hand->world:" + str(sample_list.hand_world_samples.transforms[i]) + "\n camera->marker:" + str( sample_list.camera_marker_samples.transforms[i]))
    #print("\n")

if __name__ == '__main__':
    main()
