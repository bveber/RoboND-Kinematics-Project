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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        r0, r1, r2, r3, r4, r5, r6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        # Create Modified DH parameters
        DH_matrix = {
            alpha0: 0, r0: 0, d1: 0.75, q1: q1,
            alpha1: -pi / 2., r1: 0.35, d2: 0, q2: q2 - pi / 2.,
            alpha2: 0, r2: 1.25, d3: 0, q3: q3,
            alpha3: -pi / 2., r3: -0.054, d4: 1.50, q4: q4,
            alpha4: pi / 2., r4: 0, d5: 0, q5: q5,
            alpha5: -pi / 2., r5: 0, d6: 0, q6: q6,
            alpha6: 0, r6: 0, d7: 0.303, q7: 0
        }

        # Define Modified DH Transformation matrix
        def get_transform_matrix(alpha, a, d, q):
            TF = Matrix(
                [[cos(q),               -sin(q),                0,              a],
                 [sin(q) * cos(alpha),  cos(q) * cos(alpha),    -sin(alpha),    -sin(alpha) * d],
                 [sin(q) * sin(alpha),  cos(q) * sin(alpha),    cos(alpha),     cos(alpha) * d],
                 [0,                    0,                      0,              1]])

            return TF

        T0_1 = get_transform_matrix(alpha0, r0, d1, q1).subs(DH_matrix)
        T1_2 = get_transform_matrix(alpha1, r1, d2, q2).subs(DH_matrix)
        T2_3 = get_transform_matrix(alpha2, r2, d3, q3).subs(DH_matrix)
        T3_4 = get_transform_matrix(alpha3, r3, d4, q4).subs(DH_matrix)
        T4_5 = get_transform_matrix(alpha4, r4, d5, q5).subs(DH_matrix)
        T5_6 = get_transform_matrix(alpha5, r5, d6, q6).subs(DH_matrix)
        T6_G = get_transform_matrix(alpha6, r6, d7, q7).subs(DH_matrix)
        T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

        R_correct = Matrix([[0, 0, 1.0, 0], [0, -1.0, 0, 0], [1.0, 0, 0, 0], [0, 0, 0, 1.0]])
        # Total Homogeneous Transform Between (Base) Link_0 and (End Effector) Link_7
        # With orientation correction applied
        T0_G_corrected = (T0_G * R_correct)
        print 'Transformations applied!'

        # Find end_effector rotation matrix RPY (Roll, Pitch, Yaw)
        r, p, y = symbols('r p y')

        # Roll
        rotation_X = Matrix([[1, 0, 0],
                             [0, cos(r), -sin(r)],
                             [0, sin(r), cos(r)]])
        # Pitch
        rotation_Y = Matrix([[cos(p), 0, sin(p)],
                             [0, 1, 0],
                             [-sin(p), 0, cos(p)]])
        # Yaw
        rotation_Z = Matrix([[cos(y), -sin(y), 0],
                             [sin(y), cos(y), 0],
                             [0, 0, 1]])

        # Gazebo compensation
        rotation_end_effector = rotation_X * rotation_Y * rotation_Z
        rotation_corr = rotation_Z.subs(y, pi) * rotation_Y.subs(p, -pi / 2)
        rotation_end_effector = rotation_end_effector * rotation_corr

        # Initialize service response
        joint_trajectory_list = []
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

            rotation_end_effector = rotation_end_effector.subs({'r': roll, 'p': pitch, 'y': yaw})
            end_effector = Matrix([[px], [py], [pz]])
            wrist_center = np.array(end_effector - (0.303 * rotation_end_effector[:, 2])).astype(float)

            theta1 = np.arctan2(wrist_center[1], wrist_center[0])

            side_a = 1.501
            side_b = np.sqrt(
                pow(np.sqrt(wrist_center[0] * wrist_center[0] + wrist_center[1] * wrist_center[1]) - 0.35, 2) +
                pow((wrist_center[2] - 0.75), 2))

            side_c = 1.25
            angle_a = np.arccos(max(
                [min([(side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c), 1]),
                 -1]))
            angle_b = np.arccos(max(
                [min([(side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c), 1]),
                 -1]))
            theta2 = pi / 2 - angle_a - np.arctan2(
                wrist_center[2] - 0.75,
                np.sqrt(wrist_center[0] * wrist_center[0] + wrist_center[1] * wrist_center[1]) - 0.35)

            link4_sag = 0.036
            theta3 = pi / 2 - (angle_b + link4_sag)  # account for sag in link4 of -0.054m

            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            R3_6 = np.array((R0_3.transpose() * rotation_end_effector).subs({q1: theta1, q2: theta2, q3: theta3}),
                            dtype=float)

            theta4 = np.arctan2(R3_6[2, 2], -R3_6[0, 2])
            theta5 = np.arctan2(np.sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])
            theta6 = np.arctan2(-R3_6[1, 1], R3_6[1, 0])

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
