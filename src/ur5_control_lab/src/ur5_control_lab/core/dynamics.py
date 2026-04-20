#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
dynamics.py
ur5_control_lab core module

Handles Dynamics Analysis:
1. Mass Matrix (M) Calculation
2. Coriolis Vector (C) Calculation
3. Gravity Vector (G) Calculation
"""

import numpy as np
import PyKDL
from kdl_parser_py import urdf
import rospy

class DynamicsModule:
    def __init__(self, robot_description_param='/robot_description'):
        self._init_kdl(robot_description_param)

    def _init_kdl(self, param):
        try:
            desc = rospy.get_param(param)
            success, tree = urdf.treeFromString(desc)
            if not success: raise Exception("Tree parsing failed")
            
            chain = tree.getChain("base", "wrist_3_link")
            self.gravity = PyKDL.Vector(0, 0, -9.81)
            self.dyn_solver = PyKDL.ChainDynParam(chain, self.gravity)
            self.dof = chain.getNrOfJoints()
            self.initialized = True
        except Exception as e:
            rospy.logwarn(f"Dynamics KDL Init Failed: {e}")
            self.initialized = False

    def compute_dynamics(self, q_list, qdot_list):
        """
        Computes M, C, G matrices/vectors
        Args:
            q_list: list of joint positions (rad)
            qdot_list: list of joint velocities (rad/s)
        Returns:
            dict containing 'M', 'C', 'G' as numpy arrays
        """
        if not self.initialized: return None

        q = PyKDL.JntArray(self.dof)
        qdot = PyKDL.JntArray(self.dof)
        for i in range(self.dof):
            q[i] = q_list[i]
            qdot[i] = qdot_list[i]

        # 1. Mass Matrix M
        M_kdl = PyKDL.JntSpaceInertiaMatrix(self.dof)
        self.dyn_solver.JntToMass(q, M_kdl)
        M = np.zeros((self.dof, self.dof))
        for i in range(self.dof):
            for j in range(self.dof):
                M[i,j] = M_kdl[i,j]

        # 2. Gravity Vector G
        G_kdl = PyKDL.JntArray(self.dof)
        self.dyn_solver.JntToGravity(q, G_kdl)
        G = np.zeros(self.dof)
        for i in range(self.dof):
            G[i] = G_kdl[i]

        # 3. Coriolis Vector C
        C_kdl = PyKDL.JntArray(self.dof)
        self.dyn_solver.JntToCoriolis(q, qdot, C_kdl)
        C = np.zeros(self.dof)
        for i in range(self.dof):
            C[i] = C_kdl[i]

        return {"M": M, "G": G, "C": C}
