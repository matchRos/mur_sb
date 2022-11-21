#! /usr/bin/env python3
from tf import transformations
import math
import rospy




def cartesian_controller(act_pose,set_pose_x,set_pose_y,w_target,v_target,phi_target):
    Kv = 0.5
    Ky = 0.45
    Kx = 0.15
    phi_act = transformations.euler_from_quaternion([act_pose .orientation.x,act_pose .orientation.y,act_pose .orientation.z,act_pose .orientation.w])


    e_x = (set_pose_x-act_pose.position.x)
    e_y = (set_pose_y - act_pose.position.y)
    e_local_x = math.cos(phi_act[2]) * e_x + math.sin(phi_act[2]) * e_y
    e_local_y = math.cos(phi_act[2]) * e_y - math.sin(phi_act[2]) * e_x


    u_w = w_target + v_target * Kv * e_local_y + Ky * math.sin(phi_target-phi_act[2])
    u_v = v_target * math.cos(phi_target-phi_act[2]) + Kx*e_local_x

    return u_v, u_w