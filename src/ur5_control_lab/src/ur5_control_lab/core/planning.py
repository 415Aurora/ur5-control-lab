#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
planning.py
ur5_control_lab core module

Handles Trajectory Planning:
1. Joint Space Planning (5th-order Polynomial)
2. Cartesian Space Planning (Linear Interpolation)
"""

import numpy as np

class PlanningModule:
    def __init__(self, kinematics_module):
        self.kin = kinematics_module

    def plan_joint_space(self, start_joints, end_joints, duration, steps=100):
        """
        Generates 5th-order polynomial trajectory in joint space.
        Returns: list of joint position lists
        """
        t = np.linspace(0, duration, steps)
        points = []
        
        for i in range(len(start_joints)):
            q0 = start_joints[i]
            q1 = end_joints[i]
            
            # quintic polynomial coeff: s(t) = a0 + a1t + a2t^2 + a3t^3 + a4t^4 + a5t^5
            # boundary conditions: start/end pos = q0/q1, vel=0, acc=0
            # simplified for standardized time T:
            # q(t) = q0 + (q1-q0) * (10(t/T)^3 - 15(t/T)^4 + 6(t/T)^5)
            
            traj = []
            for time_point in t:
                s = time_point / duration
                poly = 10 * s**3 - 15 * s**4 + 6 * s**5
                q = q0 + (q1 - q0) * poly
                traj.append(q)
            
            points.append(traj)
            
        # Transpose to get list of waypoints [p1, p2...]
        trajectory = []
        for i in range(steps):
            wp = [points[j][i] for j in range(len(start_joints))]
            trajectory.append(wp)
            
        return trajectory

    def plan_cartesian_space(self, start_pose_matrix, end_pose_matrix, duration, steps=50):
        """
        Generates Linear path in Cartesian space.
        Note: Requires Numerical IK which is complex to implement robustly pure python without MoveIt/IKFast.
        For this demo, we will generate the Cartesian Waypoints but we lack a robust python IK solver in this simple setup.
        
        However, to demonstrate the "Planning" aspect, we will interpolate the matrix 
        and return the Cartesian Path for visualization. 
        Execution would typically require sending these to a controller that supports Cartesian targets or solving IK.
        """
        
        # Deconstruct Start
        p0 = start_pose_matrix[:3, 3]
        # Deconstruct End
        p1 = end_pose_matrix[:3, 3]
        
        # Orientation interpolation (Slerp) is complex without scipy/quaternion lib.
        # We will Linear Interpolate Position and keep Orientation of Start (Simplified)
        
        t = np.linspace(0, 1, steps)
        trajectory_cartesian = []
        
        for s in t:
            # Lerp Position
            p = p0 + (p1 - p0) * s
            
            # Construct Waypoint Matrix
            T = np.eye(4)
            T[:3, :3] = start_pose_matrix[:3, :3] # Keep orientation
            T[:3, 3] = p
            trajectory_cartesian.append(T)
            
        return trajectory_cartesian
