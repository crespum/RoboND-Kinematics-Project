#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        
        # Define symbols for joint variables and orientation
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        roll, pitch, yaw = symbols('roll pitch yaw')

        # Define modified DH parameters
        s = {alpha0:  0,         a0:  0,         d1: 0.75,    q1: q1,
             alpha1: -1*pi/2.,   a1:  0.35,      d2: 0,       q2: q2-pi/2.,
             alpha2:  0,         a2:  1.25,      d3: 0,       q3: q3,
             alpha3: -1*pi/2.,   a3: -0.054,     d4: 1.5,     q4: q4,
             alpha4:  pi/2.,     a4:  0,         d5: 0,       q5: q5,
             alpha5: -1*pi/2.,   a5:  0,         d6: 0,       q6: q6,
             alpha6:  0,         a6:  0,         d7: 0.303,   q7: 0 }

        # Define Modified DH Transformation matrix
        def transf_matrix(alpha, a, d, q):
            tm = Matrix([[            cos(q),           -sin(q),          0,                a],
                         [ sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha),  -sin(alpha)*d],
                         [ sin(q)*sin(alpha), cos(q)*sin(alpha),   cos(alpha),   cos(alpha)*d],
                         [                 0,                 0,            0,              1]])
            return tm

        # Create individual transformation matrices
        T0_1 = transf_matrix(alpha0, a0, d1, q1).subs(s)
        T1_2 = transf_matrix(alpha1, a1, d2, q2).subs(s)
        T2_3 = transf_matrix(alpha2, a2, d3, q3).subs(s)
        T3_4 = transf_matrix(alpha3, a3, d4, q4).subs(s)
        T4_5 = transf_matrix(alpha4, a4, d5, q5).subs(s)
        T5_6 = transf_matrix(alpha5, a5, d6, q6).subs(s)
        T6_7 = transf_matrix(alpha6, a6, d7, q7).subs(s)

        # Transform from base link to end effector
        T0_7 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7
        R0_3 = (T0_1 * T1_2 * T2_3)[0:3, 0:3]

        # Extract rotation matrices from the transformation matrices

        # Define total rotation matrix
        def rot_z(angle):
            return Matrix([[ cos(angle),     -sin(angle),    0 ],
                           [ sin(angle),      cos(angle),    0 ],
                           [    0,             0,            1 ]])
        def rot_y(angle):
            return Matrix([[ cos(angle),     0,     sin(angle) ],
                           [     0,          1,          0     ],
                           [-sin(angle),     0,     cos(angle) ]])
        def rot_x(angle):
            return Matrix([[ 1,            0,            0       ],
                           [ 0,        cos(angle),   -sin(angle) ],
                           [ 0,        sin(angle),    cos(angle) ]])

        R_corr = rot_z(pi) * rot_y(-pi/2.)
        Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr   
     
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

            (roll_ee, pitch_ee, yaw_ee) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            Rrpy_val = Rrpy.evalf(subs={roll: roll_ee, yaw: yaw_ee, pitch: pitch_ee})

            wx = px - s[d7]*Rrpy_val[0, 2]
            wy = py - s[d7]*Rrpy_val[1, 2]
            wz = pz - s[d7]*Rrpy_val[2, 2]

            # Calculate ARM joint angles using Geometric IK method
            wzp = wz - s[d1]
            wxyp = sqrt(pow(wx, 2) + pow(wy, 2)) - s[a1]
            gamma = atan2(wzp, wxyp)
            delta = atan2(abs(s[a3]), s[d4])

            C = s[a2]
            B = sqrt(pow(wxyp, 2) + pow(wzp, 2))
            A = sqrt(pow(s[d4], 2) + pow(abs(s[a3]), 2))
            a = acos((pow(B, 2) + pow(C, 2) - pow(A, 2))/(2*B*C))
            b = acos((pow(A, 2) + pow(C, 2) - pow(B, 2))/(2*A*C))

            theta1 = atan2(wy, wx)
            theta2 = pi/2. - a - gamma
            theta3 = pi/2. - b - delta

            # Calculate WRIST joint angles using Geometric IK method
            R0_3_val = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6_val = R0_3_val.transpose() * Rrpy_val
            
            theta4 = atan2(R3_6_val[2, 2], -R3_6_val[0, 2])
            theta5 = atan2(sqrt(pow(R3_6_val[0, 2], 2) + pow(R3_6_val[2, 2], 2)), R3_6_val[1, 2])
            theta6 = atan2(-R3_6_val[1, 1], R3_6_val[1, 0])
        
        # Populate response for the IK request
        # In the next line replace theta1,theta2...,theta6 by your joint angle variables
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
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
