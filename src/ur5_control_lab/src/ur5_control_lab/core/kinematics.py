#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
kinematics.py
ur5_control_lab core module

Handles Kinematic Analysis:
1. Standard D-H Parameters storage
2. Transformation Matrix Calculation (FK)
3. Jacobian Calculation
"""

import numpy as np
import PyKDL
from kdl_parser_py import urdf
import rospy

class KinematicsModule:
    def __init__(self, robot_description_param='/robot_description'):
        # Standard DH Parameters for UR5 (Source: Universal Robots)
        # a: Link length, d: Link offset, alpha: Link twist, theta: Joint angle offset
        self.dh_params = [
            {"name": "Step 1 (Base -> Shoulder)", "a": 0,       "d": 0.089159, "alpha": np.pi/2, "theta_offset": 0},
            {"name": "Step 2 (Shoulder -> Elbow)", "a": -0.425,  "d": 0,        "alpha": 0,       "theta_offset": 0},
            {"name": "Step 3 (Elbow -> Wrist 1)",  "a": -0.39225, "d": 0,        "alpha": 0,       "theta_offset": 0},
            {"name": "Step 4 (Wrist 1 -> Wrist 2)","a": 0,       "d": 0.10915,  "alpha": np.pi/2, "theta_offset": 0},
            {"name": "Step 5 (Wrist 2 -> Wrist 3)","a": 0,       "d": 0.09465,  "alpha": -np.pi/2,"theta_offset": 0},
            {"name": "Step 6 (Wrist 3 -> EE)",     "a": 0,       "d": 0.0823,   "alpha": 0,       "theta_offset": 0}
        ]
        
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
                            
        # Initialize KDL Chain for verified FK/Jacobian
        self._init_kdl(robot_description_param)

    def _init_kdl(self, param):
        try:
            desc = rospy.get_param(param)
            success, tree = urdf.treeFromString(desc)
            self.chain = tree.getChain("base", "wrist_3_link")
            self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
            self.jac_solver = PyKDL.ChainJntToJacSolver(self.chain)
            self.initialized = True
        except Exception as e:
            rospy.logwarn(f"Kinematics KDL Init Failed: {e}")
            self.initialized = False

    def get_dh_table(self, current_joints=None):
        """Returns format-ready list of DH params with current Thetas"""
        table = []
        for i, param in enumerate(self.dh_params):
            theta = current_joints[i] if current_joints else 0.0
            table.append({
                "Joint": f"J{i+1}",
                "theta (rad)": theta + param['theta_offset'],
                "d (m)": param['d'],
                "a (m)": param['a'],
                "alpha (rad)": param['alpha']
            })
        return table

    def compute_fk(self, joint_positions):
        """Computes End-Effector Pose (Homogeneous Matrix 4x4)"""
        if not self.initialized: return np.eye(4)
        
        q = PyKDL.JntArray(len(joint_positions))
        for i, j in enumerate(joint_positions): q[i] = j
        
        frame = PyKDL.Frame()
        self.fk_solver.JntToCart(q, frame)
        
        # Convert to numpy 4x4
        T = np.eye(4)
        for i in range(3):
            for j in range(3):
                T[i,j] = frame.M[i,j]
            T[i,3] = frame.p[i]
        return T

    def compute_jacobian(self, joint_positions):
        """Computes Geometric Jacobian (6x6)"""
        if not self.initialized: return np.zeros((6,6))
        
        q = PyKDL.JntArray(len(joint_positions))
        for i, j in enumerate(joint_positions): q[i] = j
        
        jac = PyKDL.Jacobian(6)
        self.jac_solver.JntToJac(q, jac)
        
        # Convert to numpy
        J = np.zeros((6, 6))
        for i in range(6):
            for j in range(6):
                J[i,j] = jac[i,j]
        return J
