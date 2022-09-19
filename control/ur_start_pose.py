#!/usr/bin/env python3

import math
import sys
from turtle import pos
from nav_msgs.msg import Odometry
import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from tf import transformations
from nav_msgs.msg import Path
import tf
from bauschaum.match_lib.match_geometry import inverseTransformationMat
from controller_manager_msgs.srv import SwitchController
from inverse_kinematics import Inverse_kinematics

np.set_printoptions(precision=8,suppress=True)


def switch_controllers(start=['arm_controller'], stop=['joint_group_vel_controller']):
    rospy.wait_for_service('controller_manager/switch_controller')
    try:
        switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
        switch_controller(start_controllers=start, stop_controllers=stop, strictness=2)
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False

def switch_to_moveit():
    return switch_controllers(['arm_controller'], ['joint_group_vel_controller'])

def switch_to_velocity():
    return switch_controllers(['joint_group_vel_controller'], ['arm_controller'])


        
class robot():  
    def __init__(self, ns="mur216", group_name="UR_arm", use_moveit=True):
        rospy.init_node("ur_start")
        rospy.Subscriber('/tool0_pose', Pose, self.vel_cb)
        rospy.Subscriber('joint_states', JointState, self.joint_states_cb)
        moveit_commander.roscpp_initialize(sys.argv)
        # self.robot = moveit_commander.RobotCommander(robot_description= ns+"/robot_description") #, ns="ns")
        self.group = moveit_commander.MoveGroupCommander(group_name, robot_description= ns+"/robot_description",wait_for_servers=5.0) #, ns=ns)
        #self.joint_vel_pub = rospy.Publisher("/"+ns+"/joint_group_vel_controller/command", Float64MultiArray, queue_size=1)
        self.pos_reached = rospy.Publisher('/ur_initialized', Bool, queue_size=1)
        self.first_call = True
        self.joint_group_vel = Float64MultiArray()
        self.listener=tf.TransformListener()
        self.use_moveit = use_moveit
        self.path=Path()
        self.first_pose_rel = [0.0]*6

        self.reset()

    def reset(self):
        """Resets variables depending on ur_path and mir_pos.

        Sets:
        self.acceleration = 0.0
        self.i = 0.0
            self.joint_togoal_diff
            self.joint_goal
            self.goal
        """
        rospy.set_param("/ur_initialized", False) 
        first_call = True
        #### Wait for Initialization ####
        path = rospy.wait_for_message("ur_path", Path)
        # path = rospy.wait_for_message("ur_trajectory", Path)
        self.path=path
        request = False
        rate = rospy.Rate(1)
        while not request:
            if rospy.is_shutdown():
                return  # remaining code of function doesnt need to be called
            request = rospy.get_param('/mir_initialized', False)
            rospy.loginfo("Wait for MiR to be initialized...")
            # rospy.loginfo(f"MiR initialized: {str(request)}")
            rate.sleep()
        
        
        if first_call:  # TODO: first call always true? 
            mir_pos = rospy.wait_for_message('ground_truth', Odometry).pose.pose
            first_call = False
        # Goal-transformation from UR-Base to TCP
        t = trans()
        tcp_t_ur_euler, tcp_t_ur_quat = t.compute_transformation(path, mir_pos)
        rospy.loginfo(f"tcp_t_ur_quat: {tcp_t_ur_quat}")

        # for MoveIt: base_footprint is reference link
        tMat_tcp = transformations.euler_matrix(tcp_t_ur_euler[3], tcp_t_ur_euler[4], tcp_t_ur_euler[5], 'sxyz')
        tMat_tcp[0][3] = tcp_t_ur_quat[0]
        tMat_tcp[1][3] = tcp_t_ur_quat[1]
        tMat_tcp[2][3] = tcp_t_ur_quat[2]
        tMat_tcp=np.array(tMat_tcp)
        rospy.loginfo(f"tMat_tcp: {tMat_tcp}")
        self.listener.waitForTransform("/mur216/UR16/base_link", "/mur216/base_footprint", rospy.Time(), rospy.Duration(4.0))
        transf=self.listener.lookupTransform('/mur216/base_footprint','/mur216/UR16/base_link', rospy.Time(0))
        rospy.loginfo(f"looking up transform to base_footprint for moveit in sim. transf= {transf}")
        baseTfootprint = np.diag([1.0,1.0,1.0,1.0])
        r_baseTfootprint = transf[0]    # keine Drehung
        rospy.loginfo(f"r_baseTfootprint: {r_baseTfootprint}")
        baseTfootprint[0:3,3] = r_baseTfootprint
        rospy.loginfo(f"baseTfootprint: {baseTfootprint}")
        # tMat_ur_tcp = inverseTransformationMat(t.mirbase_T_urbase) @ tMat_tcp
        tMat_ur_tcp = baseTfootprint @ tMat_tcp
        rospy.loginfo(f"tMat_ur_tcp: {tMat_ur_tcp}")
        self.first_pose_rel = Pose(position=Point(*tMat_ur_tcp[0:3,3]), orientation=Quaternion(*transformations.quaternion_from_matrix(tMat_ur_tcp)))
        rospy.loginfo(f"first_pose_rel: {self.first_pose_rel}")


        if not self.use_moveit:
            ### Calculate IK ###
            ik = Inverse_kinematics(tcp_t_ur_quat)
            
            #Gelenkwinkelkonfigurationen
            ik.calc_solutions([],0)
            rospy.loginfo("Moegliche Loesungen wahre Pose")
            for sol in ik.solutions:
                #test_solution(sol)
                rospy.loginfo(sol)
            rospy.loginfo("")

            # test for switched goal: TODO: check if this is necessary
            ik.goal_pose = [*self.first_pose_rel.position.__reduce__()[2], *self.first_pose_rel.orientation.__reduce__()[2]]
            rospy.loginfo(f"goal_pose: {ik.goal_pose}")
            ik.solutions = []
            ik.calc_solutions([],0)
            rospy.loginfo("Moegliche Loesungen first_pose_rel")
            for sol in ik.solutions:
                #test_solution(sol)
                rospy.loginfo(sol)
            rospy.loginfo("")
            
            # Select best IK config.
            ik_solution = ik.select_solution(ik.solutions)
            
            #UR control
            self.joint_togoal_diff = ik.diff_to_goal
            self.joint_goal = ik_solution
            self.goal = tcp_t_ur_euler

            self.acceleration = 0.0
            self.i = 0.0       #TODO: wofuer?
         
    # def joint_vel(self, i, joint_goal, velocity=0.05):
    #     """Return positive/negative velocity depending on joint goal direction

    #     Args:
    #         i (integer): idx of joint
    #         joint_goal (list[float]): joint goal to reach
    #         velocity (float, optional): velocity to return. Defaults to 0.05.

    #     Returns:
    #         float: positive/negative velocity or 0 if goal reached
    #     """
    #     curr_joints = self.joint_states #[0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #self.group.get_current_joint_values()
    #     if curr_joints[i] < joint_goal[i] - 0.001:
    #         return velocity
    #     elif curr_joints[i] > joint_goal[i] + 0.001:
    #         return (-1) * velocity
    #     else:            
    #         return 0.0
        
    # def calc_accel(self, distance_to_goal):
    #     """Calculate Acceleration to accel slowly and decel before reaching goal

    #     Args:
    #         distance_to_goal (float): distance to goal (norm2 of goal minus eef_pos)

    #     Returns:
    #         float: Acceleration factor between 0 and 1
    #     """
    #     # Accelerate slowly
    #     if self.acceleration < 61:
    #         accel = self.acceleration/60
    #         self.acceleration = self.acceleration + 1
    #     else:
    #         accel = 1

    #      # Slow down if near goal pos
    #     if distance_to_goal < 0.08:
    #         decel = distance_to_goal/0.08
    #         if decel < 0.1:
    #             decel = 0.1       
    #     else:
    #         decel = 1

    #     return accel*decel
         
    # def run(self):
    #     """Publishes Joint velocities to reach EEF-goal. So that all joints approx. reach goal at same time.
    #     """
    #     # rospy.loginfo(self.i)
    #     self.i += 1.0

    #     distance_to_goal = math.sqrt(sum((self.goal[i] - self.current_ee_pos[i])**2 for i in range(3)))
    #     if not self.i%50:   # dont spam
    #         rospy.loginfo(f"distance_to_goal: {distance_to_goal}")
    #     accel = self.calc_accel(distance_to_goal)

    #     # get max Joint Diff
    #     max_joint_diff = max(map(abs, self.joint_togoal_diff))
    #     # rospy.loginfo(f"max_joint_diff: {max_joint_diff}")

    #     cmd_joint_vel = [accel * self.joint_vel(i, self.joint_goal) * abs(self.joint_togoal_diff[i]) / abs(max_joint_diff) for i in range(6)]
    #     self.joint_group_vel.data = cmd_joint_vel 
               
    #     if all(v<0.1 for v in cmd_joint_vel) and distance_to_goal < 0.05:
    #         rospy.set_param("/ur_initialized", True) 
    #         self.joint_group_vel.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #         self.joint_vel_pub.publish(self.joint_group_vel)   # TODO: warum nicht? bzw. if am Schluss? --> Restbewegung?
    #         rospy.signal_shutdown("Position reached...")
        
    #     self.joint_vel_pub.publish(self.joint_group_vel)
        
        
    def joint_states_cb(self, data):  
        a = data #a = joint_states_mixed
        self.joint_states = [a.position[2], a.position[1], a.position[0], a.position[3], a.position[4], a.position[5]]        
        
            
    def vel_cb(self, data):
        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        x = data.position.x 
        y = data.position.y
        z = data.position.z
        x_roll = euler[0]
        y_pitch = euler[1]
        z_yaw = euler[2]
        self.current_ee_pos = (x, y, z, x_roll, y_pitch, z_yaw)
        #rospy.loginfo(self.current_ee_pos)
        
        
class trans():
    def __init__(self):
        """calculate transformation for mur
        """
        #Transformation MiR-Base zu UR-Base (statisch)   
        #### actually 180.26 degrees #####
        R_z_180 = [[-0.999999, -0.004538, 0], 
                   [0.004538, -0.999999, 0],          
                   [0, 0, 1]] 
        # t_mir_ur = [0.173364, -0.102631, 0.45]
        t_mir_ur = [-0.173364, 0.102631, 0.45]  # x,y umgekehrt. TODO: besser aus tf holen
        self.mirbase_T_urbase = np.array([[R_z_180[0][0],R_z_180[0][1],R_z_180[0][2],t_mir_ur[0]],
                                    [R_z_180[1][0],R_z_180[1][1],R_z_180[1][2],t_mir_ur[1]],
                                    [R_z_180[2][0],R_z_180[2][1],R_z_180[2][2],t_mir_ur[2]],
                                    [0,0,0,1]])
            
    def compute_transformation(self, path, mir_pos):
        """Computs Transformation between UR_Base and first path pose. Distanz is calculated from mir_pos to first path pose

        Args:
            path (nav_msgs.Path): goal path
            mir_pos (geometry_msgs.Pose): current pose of Mir

        Returns:
            trans_euler, trans_quaternion
            list[float]: [x,y,z, alpha,beta,gamma], list[float]: [x,y,z, a,b,c,d]
        """
        #Tranformation World - MiR-Base
        rospy.loginfo("Zielpose")
        rospy.loginfo(path.poses[0].pose)
        #rospy.loginfo("MiR Pose")
        #rospy.loginfo(mir_pos)
        
        mir_trans = [mir_pos.position.x, mir_pos.position.y, mir_pos.position.z]
        mir_rot = [mir_pos.orientation.x, mir_pos.orientation.y, mir_pos.orientation.z, mir_pos.orientation.w]
        world_T_mirbase = transformations.quaternion_matrix(mir_rot)
        world_T_mirbase[0][3] = mir_trans[0]
        world_T_mirbase[1][3] = mir_trans[1]
        world_T_mirbase[2][3] = mir_trans[2]
        
        #rospy.loginfo("")
        #rospy.loginfo(mirbase_T_urbase)
        #rospy.loginfo("")
        
        world_t_ur = np.dot(world_T_mirbase, self.mirbase_T_urbase)
        # ur_t_world = np.linalg.inv(world_t_ur)
        ur_t_world = inverseTransformationMat(world_t_ur)

        urbase_euler = transformations.euler_from_matrix(world_t_ur)
        # urbase_quat = transformations.quaternion_from_matrix(world_t_ur)
        
        first_pose = path.poses[0].pose
        rospy.loginfo("Distanz MiR - Start")
        rospy.loginfo(f"{first_pose.position.x - world_t_ur[0][3]}, {first_pose.position.y - world_t_ur[1][3]}, {first_pose.position.z - world_t_ur[2][3]}")
        x1_world = np.matrix([[first_pose.position.x], [first_pose.position.y], [first_pose.position.z], [1]])
        #rospy.loginfo(x1_world)
        x1_urbase = ur_t_world @ x1_world
        rospy.loginfo(f"Z World: {x1_world[2]}")
        rospy.loginfo(f"Z Base: {x1_urbase[2]}")
        #rospy.loginfo("x to urbase")
        #rospy.loginfo(x1_urbase)
        tcp_t_ur_euler = [x1_urbase.item(0), x1_urbase.item(1), x1_urbase.item(2) + 0.3, 0, math.pi, -urbase_euler[2]]
        quat = transformations.quaternion_from_euler(tcp_t_ur_euler[3], tcp_t_ur_euler[4], tcp_t_ur_euler[5])
        tcp_t_ur_quat = [tcp_t_ur_euler[0], tcp_t_ur_euler[1], tcp_t_ur_euler[2], quat[0], quat[1], quat[2], quat[3]]

        return tcp_t_ur_euler, tcp_t_ur_quat
        

if __name__ == "__main__":
    use_moveit = False
    rospy.loginfo("Starting node")
    ur = robot(use_moveit=use_moveit)
    switch_to_moveit()
    print(ur.joint_goal)

    ur.group.go(ur.joint_goal, wait=True)

    ur.group.stop() 
    # rate = rospy.Rate(100)
    # if not use_moveit:
    #     switch_to_velocity()
    #     rospy.loginfo("Not using moveit")
    #     # while not rospy.is_shutdown():
    #     #     ur.run()
    #     #     rate.sleep()
    #     switch_to_moveit()
    #     rospy.loginfo(f"joint angles: {ur.joint_goal}")
    #     success = ur.group.go(ur.joint_goal, wait=True)
    #     ur.group.stop()
    #     if success is not True:
    #         rospy.logerr(f"Moveit failed pose_goal")
    #         sys.exit(1)
    #     switch_to_velocity()
    #     rospy.loginfo("UR initialized, sleeping 2s")
    #     rospy.set_param("/ur_initialized", True)
        
    # else:
    #     # MoveIt to Pose not working
    #     switch_to_moveit()
    #     rospy.loginfo("Using moveit")
    #     # pose_goal = Pose(position=Point(*ur.first_pose_rel[:3]), orientation=Quaternion(*ur.first_pose_rel[3:]))
    #     pose_goal = ur.first_pose_rel
    #     rospy.loginfo(f"Pose for MoveIT: {pose_goal}")
    #     # pose_ur_map = ur.path.poses[0].pose
    #     # pose_goal = pose_ur_map

    #     # ur.group.set_pose_reference_frame("map")
    #     pose_msg = PoseStamped()
    #     pose_msg.header.frame_id = "base_footprint"
    #     pose_msg.header.stamp = rospy.Time.now()
    #     pose_msg.pose = pose_goal
    #     ur.group.set_pose_target(pose_msg)
    #     success = ur.group.go(wait=True)
    #     rospy.loginfo(f"Moveit success: {success}")
    #     if success is not True:
    #         rospy.logerr(f"Moveit failed pose_goal: {pose_goal}")
    #         sys.exit(1)
    #     switch_to_velocity()
    #     rospy.set_param("/ur_initialized", True)
    

        