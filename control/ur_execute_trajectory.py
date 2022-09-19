#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped, Pose, Twist, PoseWithCovarianceStamped
from tf import transformations
import math
import sys
import moveit_commander
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import tf
from sensor_msgs.msg import JointState
import csv

from formation_controller.match_lib.robot_mats.jacobians.jacobian_ur_16_eef import getJacobianUr16_base_link_inertiaUr16_wrist_3_link as getJacobian
from formation_controller.match_lib.match_robots import Joints
class ur_velocity_controller():
    
    
    def __init__(self, ns="mur216", group_name="UR_arm"):
        rospy.Subscriber('ur_trajectory', Path, self.ur_trajectory_cb)
        # rospy.Subscriber('/mur_tcp_pose', Pose, self.tcp_pose_cb)   #TODO: identisch mit /tool0_pose?
        rospy.Subscriber('/tool0_pose', Pose, self.ur_pose_cb)
        # rospy.Subscriber('joint_states', JointState, self.joint_states_cb)
        self.joint_obj = Joints()
        
        self.specified_mir_vel = TwistStamped()
        rospy.Subscriber('mobile_base_controller/cmd_vel', Twist, self.mir_vel_cb)
        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.mir_pose_cb)
        
        # Subscribers Not in use:
        rospy.Subscriber("ur_path", Path, self.path_cb)
        rospy.Subscriber('mir_trajectory', Path, self.mir_trajectory_cb)
        #rospy.Subscriber('/mur/mir/odom', Odometry, self.mir_vel_odom_cb)

        self.joint_group_vel_pub = rospy.Publisher("joint_group_vel_controller/command", Float64MultiArray, queue_size=1)
        self.test_pub = rospy.Publisher("/joint_group_vel_controller/command_test", Float64MultiArray, queue_size=1)
        self.test_pub2 = rospy.Publisher("/test_publish", Float64, queue_size=1)
        self.pose_broadcaster = tf.TransformBroadcaster()
        
        #Moveit - config
        moveit_commander.roscpp_initialize(sys.argv)
        # self.group = moveit_commander.MoveGroupCommander(group_name, ns=ns, robot_description= ns+"/robot_description",wait_for_servers=5.0)

        self.reset()

    def reset(self):
        """Resets all variables
        """

        #Parameters for current TCP velocity / differential kinematics
        self.first_call = True
        self.csv_first_call = True
        self.time_n = 0.0
        self.time_n_minus_1 = 0.0
        self.pose_n = Pose()
        self.pose_n_minus_1 = Pose()
        self.w_rot = 0.0


        # rospy.wait_for_message('/mur_tcp_pose', Pose)
        rospy.loginfo("TCP-Pose not via /mur_tcp_pose. using /tool0_pose instead")
        rospy.wait_for_message('amcl_pose', Pose)
        rospy.loginfo("MiR-Pose received ...")
        rospy.wait_for_message('/tool0_pose', Pose)
        rospy.loginfo("Tool0-Pose received ...")
        
        rospy.set_param("/ur_initialized", False)   # TODO: was passiert wenn ur_start_pose zu frueh gestartet wurde? (param schon true)
        ur_request = rospy.get_param("/ur_initialized", False)
        
        while not rospy.is_shutdown() and ur_request == False:
            ur_request = rospy.get_param("/ur_initialized", False)
            if ur_request == True:
                print("UR initialized")
            else: 
                #print("Waiting for UR to be initialized...")
                pass
        rospy.loginfo("Running ...")
        
        self.control_rate = 100 #rospy.get_param("~self.control_rate")
        self.i = 1  # number of calls of run(): trajectory idx

    
    def get_distance_mir_ur(self):
        """Calculates distance from base of Mir to TCP of UR in MiR-coordinates.

        Returns:
            float: total distance
            float: distance in x
            float: distance in y
        """
        #Transformation ur_base to tcp
        trans_ur = [self.ur_pose.position.x, self.ur_pose.position.y, self.ur_pose.position.z]
        rot_ur = [self.ur_pose.orientation.x, self.ur_pose.orientation.y, self.ur_pose.orientation.z, self.ur_pose.orientation.w]
        urbase_T_urtcp = transformations.quaternion_matrix(rot_ur)
        urbase_T_urtcp[0][3] = trans_ur[0]
        urbase_T_urtcp[1][3] = trans_ur[1]
        urbase_T_urtcp[2][3] = trans_ur[2]
        
        #Transformation MiR-Base to UR-Base
        R_z_180 = [[-0.999999, -0.004538, 0], 
                    [0.004538, -0.999999, 0],
                    [0, 0, 1]] 
        t_mir_ur = [0.173364, -0.102631, 0.45]
        mirbase_T_urbase = np.array([[R_z_180[0][0],R_z_180[0][1],R_z_180[0][2],t_mir_ur[0]],
                                [R_z_180[1][0],R_z_180[1][1],R_z_180[1][2],t_mir_ur[1]],
                                [R_z_180[2][0],R_z_180[2][1],R_z_180[2][2],t_mir_ur[2]],
                                [0,0,0,1]])
        
        #Tranformation MiR-Base zu UR-TCP
        mirbase_T_urtcp = np.dot(mirbase_T_urbase,urbase_T_urtcp)
        distance = pow(pow(mirbase_T_urtcp[0][3],2) + pow(mirbase_T_urtcp[1][3],2),0.5)
        distance_x = mirbase_T_urtcp[0][3]
        distance_y = mirbase_T_urtcp[1][3]
                
        return distance, distance_x, distance_y
    
    
    
    def get_tcp_initial_vel(self):
        """Get velocity of TCP (mir+ur) induced by velocity of mir in world coordinates (no velocity of ur relative to mir)

        Returns:
            (float): (v_x, v_y)
        """
        v_trans = self.specified_mir_vel.twist.linear.x
        radius, dis_x, dis_y = self.get_distance_mir_ur()
        # filter = 1.0
        #self.w_rot = self.w_rot * (1-filter) + self.mir_trajectorie[1][i] * filter #self.specified_mir_vel.twist.angular.z * filter
        #v_rot = self.w_rot * radius
        v_rot = self.specified_mir_vel.twist.angular.z * radius
        #v_rot = self.specified_mir_vel.twist.twist.angular.z * radius
        
        #velocity vector in the mir-frame
        angle_vrot_mir = math.atan2(dis_y, dis_x)
        v_y_mir = math.cos(angle_vrot_mir) * v_rot 
        v_x_mir = -math.sin(angle_vrot_mir) * v_rot + v_trans
        self.test_pub2.publish(self.w_rot)
        
        #velocity vector in the world-frame
        mir_orientation = transformations.quaternion_matrix([self.mir_pose.orientation.x, self.mir_pose.orientation.y, self.mir_pose.orientation.z, self.mir_pose.orientation.w])
        v_x_world = mir_orientation[0][0] * v_x_mir + mir_orientation[0][1] *  v_y_mir
        v_y_world = mir_orientation[1][0] * v_x_mir + mir_orientation[1][1] *  v_y_mir
        return [v_x_world, v_y_world]
    
    
    def trajectory_velocity(self, set_pose_phi, v_target):
        """Calculates v_x, v_y by rotation angle phi

        Args:
            set_pose_phi (float): orientation of trajectory in radians
            v_target (float): target velocity of trajectory

        Returns:
            list[float]: [v_x, v_y]
        """
        # x_diff = set_pose_x - act_pose.position.x
        # y_diff = set_pose_y - act_pose.position.y
        # alpha = math.atan2(y_diff, x_diff)
        v_x = math.cos(set_pose_phi) * v_target
        v_y = math.sin(set_pose_phi) * v_target

        return [v_x, v_y]

    
    def transf_velocity_world_to_mirbase(self, final_tcp_vel_world):
        """transform velocity in world coordinates to velocity in mir coordinates. (Approx. the same as in UR coordinates)

        Args:
            final_tcp_vel_world (list[float]): [v_x, v_y] velocity of TCP in world coordinates

        Returns:
            list[float]: [v_x_mir, v_y_mir] velocity of TCP in MIR-coordinates
        """
        mir_orientation = transformations.euler_from_quaternion([self.mir_pose.orientation.x, self.mir_pose.orientation.y, self.mir_pose.orientation.z, self.mir_pose.orientation.w])
        mir_rot_about_z = mir_orientation[2]
        
        v_x_mir = -math.cos(mir_rot_about_z) * final_tcp_vel_world[0] - math.sin(math.pi - mir_rot_about_z) * final_tcp_vel_world[1]
        v_y_mir = math.sin(mir_rot_about_z) * final_tcp_vel_world[0] + math.cos(math.pi - mir_rot_about_z) * final_tcp_vel_world[1]
        #wenn die Verdrehung von mir zu urbase von 0,26grad nicht betrachtet wird ist die korrekturgeschwindigkeit im MiR-KS gleich der im UR-KS
        return [v_x_mir, v_y_mir]

    
    def differential_inverse_kinematics_ur(self, velocities):
        """Calculate velocities of joints from cartesian TCP-Velocities

        Args:
            velocities (list[float]): dim6

        Returns:
            Float64MultiArray: joint_group_vel
        """
        joint_group_vel = Float64MultiArray()
        # current_joints = self.joint_states#self.group.get_current_joint_values()
        # # jm = np.array(self.group.get_jacobian_matrix(current_joints))
        # jm = getJacobian(current_joints)
        jm = self.joint_obj.getJacobian()
        jacobian_matrix = np.matrix(jm)
        jacobian_inverse = np.linalg.inv(jacobian_matrix)
        
        cartesian_velocities_matrix = np.matrix.transpose(np.matrix(velocities))
          
        joint_velocities = jacobian_inverse @ cartesian_velocities_matrix
        joint_group_vel.data = (np.asarray(joint_velocities)).flatten()
        
        return joint_group_vel


    def position_controller(self, target_pos_x, target_pos_y,actual_pos_x, actual_pos_y, K_p=0.1):
        """Proportional-controller. Calculates error in x,y and returns controller-output

        Args:
            target_pos_x (float): goal pos
            target_pos_y (float): goal pos
            actual_pos_x (float): current pos
            actual_pos_y (float): current pos
            K_p (float, optional): Proportional Term. Defaults to 0.1.

        Returns:
            (K_p*e_x, K_p*e_y, distance, e_x, e_y)
        """  
        e_x = target_pos_x - actual_pos_x   
        e_y = target_pos_y - actual_pos_y
        distance = pow(pow(e_x, 2) + pow(e_y, 2), 0.5)
        return K_p*e_x, K_p*e_y, distance, e_x, e_y      
        

        
    def run(self):
        """calculates joint velocities by cartesian position error and trajectory velocity. Takes current MiR velocity into account. Trajectory velocity proportional to control rate.
        """
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown() and self.i < len(self.trajectorie[3]):
            act_pose        = self.tcp_pose
            set_pose_x      = self.trajectorie[0][self.i]
            set_pose_y      = self.trajectorie[1][self.i]
            set_pose_phi    = self.trajectorie[2][self.i]
            v_target        = self.trajectorie[3][self.i] * self.control_rate
            w_target        = self.trajectorie[4][self.i] * self.control_rate
            
            #position controller
            u_x, u_y, distance, dis_x, dis_y = self.position_controller(set_pose_x, set_pose_y, act_pose.position.x, act_pose.position.y)
            
            tcp_initial_vel = self.get_tcp_initial_vel()    # only the part induced by mir to world velocity 
            print("Durchlauf: " + str(self.i))
            target_tcp_vel = self.trajectory_velocity(set_pose_phi, v_target)

            # Goal velocity of ur relative to mir:
            final_tcp_vel_world = [target_tcp_vel[0] - tcp_initial_vel[0] + u_x, target_tcp_vel[1] - tcp_initial_vel[1] + u_y]
            final_tcp_vel_mir_base = self.transf_velocity_world_to_mirbase(final_tcp_vel_world)
            
            #TCP velocity in ur_base_link
            tcp_vel_ur = [final_tcp_vel_mir_base[0], final_tcp_vel_mir_base[1], 0, 0, 0, 0]
            rospy.loginfo("tcp_vel_ur: x,y=" + str(tcp_vel_ur[0]) + "," + str(tcp_vel_ur[1]))
            joint_group_vel = self.differential_inverse_kinematics_ur(tcp_vel_ur)
            rospy.loginfo("joint_group_vel: " + str(joint_group_vel.data)+"\nfor jointstates: "+str(self.joint_obj.q))

            #publish joint velocities
            self.target_pose_broadcaster([set_pose_x,set_pose_y,set_pose_phi])
            self.test_pub.publish(joint_group_vel)
            self.joint_group_vel_pub.publish(joint_group_vel)

            print(self.i, distance, dis_x, dis_y)
            # if self.csv_first_call:
            #     with open('/home/rosmatch/Dokumente/tolerance_no_con_2_3.csv', 'w') as test:
            #         writer = csv.writer(test)
            #         writer.writerow([self.i, distance, dis_x, dis_y])
            #         self.csv_first_call = False
            # else:  
            #     with open('/home/rosmatch/Dokumente/tolerance_no_con_2_3.csv', 'a') as test:
            #         writer = csv.writer(test)
            #         writer.writerow([self.i, distance, dis_x, dis_y])
            
            self.i += 1

            rate.sleep()
            
    
    def target_pose_broadcaster(self,target_pose):
        frame_id = "tool0_target"
        self.pose_broadcaster.sendTransform((target_pose[0], target_pose[1], 0),
                     transformations.quaternion_from_euler(0, 0, target_pose[2]),
                     rospy.Time.now(), frame_id, "map")
    
    # # Using Joints() instead because of false joints if via idx only (especially if using mur instead of ur)
    # def joint_states_cb(self, data):  
    #     a = data #a = joint_states_mixed
    #     self.joint_states = [a.position[2], a.position[1], a.position[0], a.position[3], a.position[4], a.position[5]]
    
    
    def ur_trajectory_cb(self,Path):
        """Vorgegebene Trajektorie
        TODO: was passiert bei Neuvorgabe Trajektorie?
            in run(): springt von idx i der alten auf idx i der neuen Trajektorie?
        """
        trajectory_x = []
        trajectory_y = []
        trajectory_phi = []
        path_len = len(Path.poses)
        for i in range(0,path_len-1):
            trajectory_x.append(Path.poses[i].pose.position.x)
            trajectory_y.append(Path.poses[i].pose.position.y)
            phi = math.atan2(Path.poses[i+1].pose.position.y-Path.poses[i].pose.position.y,Path.poses[i+1].pose.position.x-Path.poses[i].pose.position.x)
            trajectory_phi.append(phi)
        
        trajectory_v = [0.0]
        trajectory_w = [0.0]
        for i in range(1,path_len-2):
            trajectory_v.append(math.sqrt((trajectory_x[i+1]-trajectory_x[i])**2 + (trajectory_y[i+1]-trajectory_y[i])**2 ))
            trajectory_w.append(trajectory_phi[i+1]-trajectory_phi[i])

        self.trajectorie = [trajectory_x, trajectory_y, trajectory_phi, trajectory_v, trajectory_w]
        rospy.loginfo("ur trajectory received")
           
    def mir_vel_cb(self, data):
        """cmd_vel wird verwendet
        TODO: stattdessen wahre MiR-Geschwindigkeit?
        """

        self.specified_mir_vel.twist=data
 
    def tcp_pose_cb(self, data):
        """Actual tcp pose. Topic not published?
        """
        self.tcp_pose = data
        
    def mir_pose_cb(self, data=PoseWithCovarianceStamped()):
        """Nur fuer orientation. Transformationen zwischen v_x_mir und v_x_world.
        """
        self.mir_pose = data.pose.pose
        
    def ur_pose_cb(self, data):
        """Verwendet in "get_distance_mir_ur() TODO: tcp_pose jetzt hier mit definiert"
        """
        self.ur_pose = data
        self.tcp_pose = data

    # WIRD NICHT VERWENDET:
    ############################
    def velocity_controller(self, v_target):
        """Sets control rate in proportion to the target velocity. r_new = r_old*(v_target/v + 2)/3.
        NOT IN USE

        Args:
            v_target (float): absolute target velocity
        """
        if self.first_call:
            self.pose_n = self.tcp_pose
            self.time_n = rospy.get_time()
            self.first_call = False
        else:
            self.time_n_minus_1 = self.time_n
            self.pose_n_minus_1 = self.pose_n
            self.pose_n = self.tcp_pose
            self.time_n = rospy.get_time()
            time_diff = self.time_n - self.time_n_minus_1

            x_vel = (self.pose_n.position.x - self.pose_n_minus_1.position.x)/time_diff
            y_vel = (self.pose_n.position.y - self.pose_n_minus_1.position.y)/time_diff
            velocity = pow(pow(x_vel,2) + pow(y_vel,2),0.5)
            
            if velocity == 0:
                v_target_relativ = 1.0
            else:
                v_target_relativ = abs(v_target) / velocity 
                
            actual_goal_rate = self.control_rate * v_target_relativ
            self.control_rate  = (actual_goal_rate + 2 * self.control_rate) / 3 #muted_rate

    def path_cb(self, data):
        """Wird nicht verwendet
        """
        self.ur_path = data
    
    def mir_trajectory_cb(self,Path):
        """Nicht gebraucht, da dies vorgegebener Pfad, stattdessen wahreer Pfad

        Args:
            Path (_type_): _description_
        """
        trajectory_x = []
        trajectory_y = []
        trajectory_phi = []
        path_len = len(Path.poses)
        for i in range(0,path_len-1):
            trajectory_x.append(Path.poses[i].pose.position.x)
            trajectory_y.append(Path.poses[i].pose.position.y)
            phi = math.atan2(Path.poses[i+1].pose.position.y-Path.poses[i].pose.position.y,Path.poses[i+1].pose.position.x-Path.poses[i].pose.position.x)
            trajectory_phi.append(phi)
        
        trajectory_v = [0.0]
        trajectory_w = [0.0]
        for i in range(1,path_len-2):
            trajectory_v.append(math.sqrt((trajectory_x[i+1]-trajectory_x[i])**2 + (trajectory_y[i+1]-trajectory_y[i])**2 ))
            trajectory_w.append(trajectory_phi[i+1]-trajectory_phi[i])

        self.mir_trajectorie = [trajectory_v, trajectory_w]
        rospy.loginfo("mir trajectory received")
        
    def mir_vel_odom_cb(self, data):
        """Wird nicht verwendet. Stattdessen mir_vel_cb
        """
        self.specified_mir_vel = data   
        
if __name__ == "__main__":
    rospy.init_node("ur_velocity_controller")
    velocity = ur_velocity_controller()
    velocity.run()