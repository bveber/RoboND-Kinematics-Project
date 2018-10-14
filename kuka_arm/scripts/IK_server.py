#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author:

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np
import matplotlib.pyplot as plt

ERRORS = []


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        DH_matrix = {
            'alpha0': 0, 'r0': 0, 'd1': 0.75,
            'alpha1': -np.pi / 2., 'r1': 0.35, 'd2': 0,
            'alpha2': 0, 'r2': 1.25, 'd3': 0,
            'alpha3': -np.pi / 2., 'r3': -0.054, 'd4': 1.50,
            'alpha4': np.pi / 2., 'r4': 0, 'd5': 0,
            'alpha5': -np.pi / 2., 'r5': 0, 'd6': 0,
            'alpha6': 0, 'r6': 0, 'd7': 0.303
        }

        # Define Modified DH Transformation matrix
        def get_transform_matrix(alpha, a, d, q):
            TF = np.matrix(
                [[np.cos(q), -np.sin(q), 0, a],
                 [np.sin(q) * np.cos(alpha), np.cos(q) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
                 [np.sin(q) * np.sin(alpha), np.cos(q) * np.sin(alpha), np.cos(alpha),  np.cos(alpha) * d],
                 [0, 0, 0, 1]])

            return TF

        # Roll
        def get_rotX(r):
            return np.matrix([[1, 0, 0],
                             [0, np.cos(r), -np.sin(r)],
                             [0, np.sin(r), np.cos(r)]])

        # Pitch
        def get_rotY(p):
            return np.matrix([[np.cos(p), 0, np.sin(p)],
                             [0, 1, 0],
                             [-np.sin(p), 0, np.cos(p)]])

        # Yaw
        def get_rotZ(y):
            return np.matrix([[np.cos(y), -np.sin(y), 0],
                             [np.sin(y), np.cos(y), 0],
                             [0, 0, 1]])

        # Initialize service response
        joint_trajectory_list = []
        rotation_corr = get_rotZ(np.pi) * get_rotY(-np.pi / 2)
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            # rotation_end_effector.subs({'r': roll, 'p': pitch, 'y': yaw})
            # end_effector = Matrix([[px], [py], [pz]])
            rotation_end_effector = get_rotX(roll) * get_rotY(pitch) * get_rotZ(yaw) * rotation_corr
            end_effector = np.matrix([[px], [py], [pz]])
            wrist_center = end_effector - (DH_matrix['d7'] * rotation_end_effector[:, 2])
            theta1 = np.arctan2(wrist_center[1], wrist_center[0])

            r1 = DH_matrix['r1']
            d1 = DH_matrix['d1']
            side_a = 1.501
            side_b = np.sqrt(
                (np.sqrt(wrist_center[0] ** 2 + wrist_center[1] ** 2) - r1) ** 2 +
                (wrist_center[2] - d1) ** 2)

            side_c = 1.25
            angle_a = np.arccos(max(
                [min([(side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c), 1]),
                 -1]))
            angle_b = np.arccos(max(
                [min([(side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c), 1]),
                 -1]))
            theta2 = np.pi / 2 - angle_a - np.arctan2(
                wrist_center[2] - d1,
                np.sqrt(wrist_center[0] * wrist_center[0] + wrist_center[1] * wrist_center[1]) - r1)

            link4_sag = 0.036
            theta3 = np.pi / 2 - (angle_b + link4_sag)  # account for sag in link4 of -0.054m

            T0_1 = get_transform_matrix(DH_matrix['alpha0'], DH_matrix['r0'], DH_matrix['d1'], theta1)
            T1_2 = get_transform_matrix(DH_matrix['alpha1'], DH_matrix['r1'], DH_matrix['d2'], theta2 - np.pi/2)
            T2_3 = get_transform_matrix(DH_matrix['alpha2'], DH_matrix['r2'], DH_matrix['d3'], theta3)

            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            R3_6 = np.array(R0_3.transpose() * rotation_end_effector, dtype=float)

            theta4 = np.arctan2(R3_6[2, 2], -R3_6[0, 2])
            theta5 = np.arctan2(np.sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])
            theta6 = np.arctan2(-R3_6[1, 1], R3_6[1, 0])

            T3_4 = get_transform_matrix(DH_matrix['alpha3'], DH_matrix['r3'], DH_matrix['d4'], theta4)
            T4_5 = get_transform_matrix(DH_matrix['alpha4'], DH_matrix['r4'], DH_matrix['d5'], theta5)
            T5_6 = get_transform_matrix(DH_matrix['alpha5'], DH_matrix['r5'], DH_matrix['d6'], theta6)
            T6_G = get_transform_matrix(DH_matrix['alpha6'], DH_matrix['r6'], DH_matrix['d7'], 0)
            T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

            R_correct = np.array([[0, 0, 1.0, 0], [0, -1.0, 0, 0], [1.0, 0, 0, 0], [0, 0, 0, 1.0]])
            # Total Homogeneous Transform Between (Base) Link_0 and (End Effector) Link_7
            # With orientation correction applied
            T0_G_corrected = (T0_G * R_correct)

            FK = T0_G_corrected

            calculated_EE = [FK[0, 3], FK[1, 3], FK[2, 3]]

            # Find FK EE error
            if not (sum(calculated_EE) == 3):
                ee_x_e = abs(calculated_EE[0] - px)
                ee_y_e = abs(calculated_EE[1] - py)
                ee_z_e = abs(calculated_EE[2] - pz)
                ee_offset = sqrt(ee_x_e ** 2 + ee_y_e ** 2 + ee_z_e ** 2)
                # print ("\nEnd effector error for x position is: %04.8f" % float(ee_x_e))
                # print ("End effector error for y position is: %04.8f" % float(ee_y_e))
                # print ("End effector error for z position is: %04.8f" % float(ee_z_e))
                ERRORS.append(ee_offset)
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = np.array([theta1, theta2, theta3, theta4, theta5, theta6]).astype(float)
            joint_trajectory_list.append(joint_trajectory_point)
        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
