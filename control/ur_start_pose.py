#!/usr/bin/env python3

import math
import sys
from turtle import pos

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

class inversekinematics():
    def __init__(self, pose,ns="mur216", group_name="UR_arm"):
        # Eingabe der Zielposition
        self.goal_pose = pose 
        rospy.loginfo("IK Pose: ")
        rospy.loginfo(pose)
        # moveit_commander.roscpp_initialize(sys.argv)
        # # group_name = "manipulator"
        # moveit_commander.MoveGroupCommander(group_name, ns=ns, robot_description= ns+"/robot_description",wait_for_servers=5.0)
        joint_states_mixed = rospy.wait_for_message('joint_states', JointState)
        self.joint_values = [joint_states_mixed.position[2], joint_states_mixed.position[1], joint_states_mixed.position[0], joint_states_mixed.position[3], joint_states_mixed.position[4], joint_states_mixed.position[5]]
        #self.joint_values = self.group.get_current_joint_values()

        # DH Paramter
        self.d1 = 0.1807
        self.d2 = 0.0
        self.d3 = 0.0
        self.d4 = 0.17415
        self.d5 = 0.11985
        self.d6 = 0.11655

        self.d = [0,self.d1,self.d2,self.d3,self.d4,self.d5,self.d6]

        self.a0 = 0
        self.a1 = 0
        self.a2 = -0.4784
        self.a3 = -0.36
        self.a4 = 0
        self.a5 = 0
        self.a6 = 0

        self.a = [self.a0,self.a1,self.a2,self.a3,self.a4,self.a5,self.a6]

        self.alpha_0 = 0
        self.alpha_1 = math.pi / 2
        self.alpha_2 = 0
        self.alpha_3 = 0
        self.alpha_4 = math.pi / 2
        self.alpha_5 = -math.pi / 2
        self.alpha_6 = 0

        self.alpha = [self.alpha_0,self.alpha_1,self.alpha_2,self.alpha_3,self.alpha_4,self.alpha_5,self.alpha_6]

        self.solutions = []
        self.diff_to_goal = []  # difference: current joint angles to goal
        
        #Gelenkwinkel und Transformationen
        self.T0_6 = self.calc_T_0_6()
        self.T6_0 = self.getInverseT(self.T0_6)


    # Transformation KS0 zu KS6
    def calc_T_0_6(self):
        """Transformation KS0 to KS6

        Returns:
            numpy.array(4,4): Transformation Matrix
        """
        rot_twist = [self.goal_pose[3], self.goal_pose[4], self.goal_pose[5], self.goal_pose[6]]
        # correct_rot = transformations.quaternion_about_axis(math.pi, (0, 0, 1))
        correct_rot = transformations.quaternion_about_axis(0, (0, 0, 1))
        rot = transformations.quaternion_multiply(rot_twist, correct_rot)
        self.t0_6 = transformations.quaternion_matrix(rot)
        self.t0_6[0][3] = self.goal_pose[0]
        self.t0_6[1][3] = self.goal_pose[1]
        self.t0_6[2][3] = self.goal_pose[2]
        rospy.loginfo(self.t0_6)
        return self.t0_6#np.around(self.t0_6, 10)


    # Transformation KS6 zu KS0
    def getInverseT(self, T_0_6):
        """get inverse Transformation Matrix

        Args:
            T_0_6 (numpy.array(4,4)): Transformation Matrix

        Returns:
            numpy.array(4,4): inverse Transformation Matrix
        """
        return np.linalg.inv(T_0_6)

    def calc_T_ip_i(self, qi,i):
        return np.matrix([[math.cos(qi), -math.sin(qi), 0, self.a[i-1]],
                        [math.sin(qi) * math.cos(self.alpha[i-1]), math.cos(qi) * math.cos(self.alpha[i-1]), -math.sin(self.alpha[i-1]),
                        -math.sin(self.alpha[i-1]) * self.d[i]],
                        [math.sin(qi) * math.sin(self.alpha[i-1]), math.cos(qi) * math.sin(self.alpha[i-1]), math.cos(self.alpha[i-1]),
                        math.cos(self.alpha[i-1]) * self.d[i]],
                        [0, 0, 0, 1]])

    def calc_T_1_4(self, q1, q5, q6):
        T0_1 = np.matrix([[math.cos(q1), -math.sin(q1), 0, self.a0],
                        [math.sin(q1) * math.cos(self.alpha_0), math.cos(q1) * math.cos(self.alpha_0), -math.sin(self.alpha_0),
                        -math.sin(self.alpha_0) * self.d1],
                        [math.sin(q1) * math.sin(self.alpha_0), math.cos(q1) * math.sin(self.alpha_0), math.cos(self.alpha_0),
                        math.cos(self.alpha_0) * self.d1],
                        [0, 0, 0, 1]])
        T4_5 = np.matrix([[math.cos(q5), -math.sin(q5), 0, self.a4],
                        [math.sin(q5) * math.cos(self.alpha_4), math.cos(q5) * math.cos(self.alpha_4), -math.sin(self.alpha_4),
                        -math.sin(self.alpha_4) * self.d5],
                        [math.sin(q5) * math.sin(self.alpha_4), math.cos(q5) * math.sin(self.alpha_4), math.cos(self.alpha_4),
                        math.cos(self.alpha_4) * self.d5],
                        [0, 0, 0, 1]])
        T5_6 = np.matrix([[math.cos(q6), -math.sin(q6), 0, self.a5],
                        [math.sin(q6) * math.cos(self.alpha_5), math.cos(q6) * math.cos(self.alpha_5), -math.sin(self.alpha_5),
                        -math.sin(self.alpha_5) * self.d6],
                        [math.sin(q6) * math.sin(self.alpha_5), math.cos(q6) * math.sin(self.alpha_5), math.cos(self.alpha_5),
                        math.cos(self.alpha_5) * self.d6],
                        [0, 0, 0, 1]])
        
        #T0_1 = calc_T_ip_i(q1,1)
        #T4_5 = calc_T_ip_i(q5,5)
        #T5_6 = calc_T_ip_i(q6,6)
        
        T1_0 = np.linalg.inv(T0_1)
        T5_4 = np.linalg.inv(T4_5)
        T6_5 = np.linalg.inv(T5_6)

        return T1_0 * self.T0_6 * T6_5 * T5_4


    # Berechnung der Gelenkwinkel
    def calc_q1(self):
        P0_5 = self.T0_6 * np.matrix.transpose(np.matrix([0, 0, -self.d6, 1]))
        return [
            math.atan2(P0_5[1], P0_5[0]) + math.acos(self.d4 / math.sqrt(P0_5[0] * P0_5[0] + P0_5[1] * P0_5[1])) + math.pi / 2,
            math.atan2(P0_5[1], P0_5[0]) - math.acos(self.d4 / math.sqrt(P0_5[0] * P0_5[0] + P0_5[1] * P0_5[1])) + math.pi / 2]


    def calc_q2(self, q3, T1_4):
        P1_4x = T1_4.item(3)
        P1_4z = T1_4.item(11)
        P1_4xz = math.sqrt(P1_4x * P1_4x + P1_4z * P1_4z)
        return [math.atan2(-P1_4z, -P1_4x) - math.asin(((-1) * self.a3 * math.sin(q3)) / P1_4xz)]


    def calc_q3(self, T1_4):
        P1_4x = T1_4.item(3)
        P1_4z = T1_4.item(11)
        P1_4xz = math.sqrt(P1_4x * P1_4x + P1_4z * P1_4z)   # TODO: P1_4xz² is sufficient (see below)

        # TODO: check if value is in domain ("Value Error: math domain error")
        return [-math.acos((P1_4xz * P1_4xz - self.a2 * self.a2 - self.a3 * self.a3) / (2 * self.a2 * self.a3)),
                math.acos((P1_4xz * P1_4xz - self.a2 * self.a2 - self.a3 * self.a3) / (2 * self.a2 * self.a3))]


    def calc_q4(self, q2, q3, T1_4):
        T1_2 = np.matrix([[math.cos(q2), -math.sin(q2), 0, self.a1],
                        [math.sin(q2) * math.cos(self.alpha_1), math.cos(q2) * math.cos(self.alpha_1), -math.sin(self.alpha_1),
                        -math.sin(self.alpha_1) * self.d2],
                        [math.sin(q2) * math.sin(self.alpha_1), math.cos(q2) * math.sin(self.alpha_1), math.cos(self.alpha_1),
                        math.cos(self.alpha_1) * self.d2],
                        [0, 0, 0, 1]])
        T2_3 = np.matrix([[math.cos(q3), -math.sin(q3), 0, self.a2],
                        [math.sin(q3) * math.cos(self.alpha_2), math.cos(q3) * math.cos(self.alpha_2), -math.sin(self.alpha_2),
                        -math.sin(self.alpha_2) * self.d3],
                        [math.sin(q3) * math.sin(self.alpha_2), math.cos(q3) * math.sin(self.alpha_2), math.cos(self.alpha_2),
                        math.cos(self.alpha_2) * self.d3],
                        [0, 0, 0, 1]])
        T2_1 = np.linalg.inv(T1_2)
        T3_2 = np.linalg.inv(T2_3)
        T3_4 = T3_2 * T2_1 * T1_4
        return [math.atan2(T3_4.item(4), T3_4.item(0))]


    def calc_q5(self, q1):
        return [math.acos((self.T0_6[0, 3] * math.sin(q1) - self.T0_6[1, 3] * math.cos(q1) - self.d4) / self.d6),
                -math.acos((self.T0_6[0, 3] * math.sin(q1) - self.T0_6[1, 3] * math.cos(q1) - self.d4) / self.d6)]


    def calc_q6(self, q1, q5):
        return [math.atan2((-self.T6_0.item(4) * math.sin(q1) + self.T6_0.item(5) * math.cos(q1)) / math.sin(q5),
                        (self.T6_0.item(0) * math.sin(q1) - self.T6_0.item(1) * math.cos(q1)) / math.sin(q5))]


    def calc_solutions(self, solution=[], i=0, **kwargs):
        """Recursive function. Calculates list with all possible solutions of the joint angles (self.solutions: list[list[float]])

        Args:
            solution (list[float]): current solution
            i (int): index of solution step
        """
        if i == 0:
            for q1 in self.calc_q1():
                solution = [None]*6
                solution[0] = q1
                self.calc_solutions(list(solution),i+1)
        elif i == 1:
            for q5 in self.calc_q5(solution[0]):
                solution[4] = q5
                self.calc_solutions(list(solution),i+1)     
        elif i == 2:
            for q6 in self.calc_q6(solution[0],solution[4]):
                solution[5] = q6
                self.calc_solutions(list(solution),i+1)
        elif i == 3:
            T1_4 = self.calc_T_1_4(solution[0], solution[4], solution[5])
            for q3 in self.calc_q3(T1_4):
                solution[2] = q3
                self.calc_solutions(list(solution),i+1,T = T1_4)
        elif i == 4:
            T1_4 = kwargs.get('T', None)
            for q2 in self.calc_q2(solution[2], T1_4):
                solution[1] = q2
                self.calc_solutions(list(solution),i+1,T = T1_4)
        elif i == 5:
            T1_4 = kwargs.get('T', None)
            for q4 in self.calc_q4(solution[1], solution[2], T1_4):
                solution[3] = q4
                self.calc_solutions(list(solution),i+1)
        elif i == 6:
            self.solutions.append(solution)


    def test_solution(self, solution):
        A = self.calc_T_ip_i(solution[0],1)
        for i in range(1,6):
            A = A*self.calc_T_ip_i(solution[i],i+1)
        rospy.loginfo(A-self.T0_6)


    def solution_min_diff(self, solutions, difference_solutions, pref):
        """Returns solution with min sum joint_angle difference

        Args:
            solution (list[list[float]]): possible solutions. dim:(:,6)
            difference_solutions (_type_): _description_
            pref (_type_): _description_

        Returns:
            _type_: _description_
        """
        min_joint_diff = None
        selected_solution = None
        for i in pref:
            joint_diff_tot = 0
            for k in range(6):
                joint_diff_tot = joint_diff_tot + abs(difference_solutions[i][k])
            if (min_joint_diff is None or joint_diff_tot < min_joint_diff):
                min_joint_diff = joint_diff_tot
                selected_solution = i
            else:
                pass 
        self.diff_to_goal = difference_solutions[selected_solution] 
        return solutions[selected_solution]


    def select_solution(self, solutions=None):
        """selects solution from the 8 solutions. depending on the value of q2 and the joint angle difference.

        Args:
            solution (list[list[float]], optional): size: [8,6]. solutions of ik. Defaults to self.solutions.

        Returns:
            list[float]: chosen solution
        """
        if solutions==None:
            solutions=self.solutions

        # First selections depending on value of q2
        first = []
        sec = []
        third = []
        for i in range(8):
            if (-3.0) / 4.0 * math.pi < solutions[i][1] < (-1.0) / 4.0 * math.pi: #and abs(solutions[i][0]) < math.pi/2:
                first.append(i)
            elif (-1.0) * math.pi < solutions[i][1] < 0.0: # and abs(solutions[i][0]) < math.pi/2:
                sec.append(i)
            else:
                third.append(i)

        # Final selection from previous selection uses difference between solution and current joint angle 
        difference_solutions = []   # TODO: calc in solution_min_diff?
        for i in range(8):
            temp_diff_array = []
            for k in range(6):
                temp_diff_array.append(solutions[i][k] - self.joint_values[k])
            difference_solutions.append(temp_diff_array)

        if first:
            a = self.solution_min_diff(solutions, difference_solutions, first)
            return a
        elif sec:
            a = self.solution_min_diff(solutions, difference_solutions, sec)
            return a
        elif third:
            rospy.loginfo("only third solutions. q2>0 or q2<-pi")
            a = self.solution_min_diff(solutions, difference_solutions, third)
            return a
        else:
            rospy.logerr("Error in select_solution")
        
        
        
class robot():  
    def __init__(self, ns="mur216", group_name="UR_arm", use_moveit=True):
        rospy.init_node("ur_start")
        rospy.Subscriber('/tool0_pose', Pose, self.vel_cb)
        rospy.Subscriber('joint_states', JointState, self.joint_states_cb)
        moveit_commander.roscpp_initialize(sys.argv)
        # self.robot = moveit_commander.RobotCommander(robot_description= ns+"/robot_description") #, ns="ns")
        self.group = moveit_commander.MoveGroupCommander(group_name, robot_description= ns+"/robot_description",wait_for_servers=5.0) #, ns=ns)
        self.joint_vel_pub = rospy.Publisher("/"+ns+"/joint_group_vel_controller/command", Float64MultiArray, queue_size=1)
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
            mir_pos = rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped).pose.pose
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
            ik = inversekinematics(tcp_t_ur_quat)
            
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
         
    def joint_vel(self, i, joint_goal, velocity=0.05):
        """Return positive/negative velocity depending on joint goal direction

        Args:
            i (integer): idx of joint
            joint_goal (list[float]): joint goal to reach
            velocity (float, optional): velocity to return. Defaults to 0.05.

        Returns:
            float: positive/negative velocity or 0 if goal reached
        """
        curr_joints = self.joint_states #[0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #self.group.get_current_joint_values()
        if curr_joints[i] < joint_goal[i] - 0.001:
            return velocity
        elif curr_joints[i] > joint_goal[i] + 0.001:
            return (-1) * velocity
        else:            
            return 0.0
        
    def calc_accel(self, distance_to_goal):
        """Calculate Acceleration to accel slowly and decel before reaching goal

        Args:
            distance_to_goal (float): distance to goal (norm2 of goal minus eef_pos)

        Returns:
            float: Acceleration factor between 0 and 1
        """
        # Accelerate slowly
        if self.acceleration < 61:
            accel = self.acceleration/60
            self.acceleration = self.acceleration + 1
        else:
            accel = 1

         # Slow down if near goal pos
        if distance_to_goal < 0.08:
            decel = distance_to_goal/0.08
            if decel < 0.1:
                decel = 0.1       
        else:
            decel = 1

        return accel*decel
         
    def run(self):
        """Publishes Joint velocities to reach EEF-goal. So that all joints approx. reach goal at same time.
        """
        # rospy.loginfo(self.i)
        self.i += 1.0

        distance_to_goal = math.sqrt(sum((self.goal[i] - self.current_ee_pos[i])**2 for i in range(3)))
        if not self.i%50:   # dont spam
            rospy.loginfo(f"distance_to_goal: {distance_to_goal}")
        accel = self.calc_accel(distance_to_goal)

        # get max Joint Diff
        max_joint_diff = max(map(abs, self.joint_togoal_diff))
        # rospy.loginfo(f"max_joint_diff: {max_joint_diff}")

        cmd_joint_vel = [accel * self.joint_vel(i, self.joint_goal) * abs(self.joint_togoal_diff[i]) / abs(max_joint_diff) for i in range(6)]
        self.joint_group_vel.data = cmd_joint_vel 
               
        if all(v<0.1 for v in cmd_joint_vel) and distance_to_goal < 0.05:
            rospy.set_param("/ur_initialized", True) 
            self.joint_group_vel.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.joint_vel_pub.publish(self.joint_group_vel)   # TODO: warum nicht? bzw. if am Schluss? --> Restbewegung?
            rospy.signal_shutdown("Position reached...")
        
        self.joint_vel_pub.publish(self.joint_group_vel)
        
        
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
    rate = rospy.Rate(100)
    if not use_moveit:
        switch_to_velocity()
        rospy.loginfo("Not using moveit")
        # while not rospy.is_shutdown():
        #     ur.run()
        #     rate.sleep()
        switch_to_moveit()
        rospy.loginfo(f"joint angles: {ur.joint_goal}")
        success = ur.group.go(ur.joint_goal, wait=True)
        ur.group.stop()
        if success is not True:
            rospy.logerr(f"Moveit failed pose_goal")
            sys.exit(1)
        switch_to_velocity()
        rospy.loginfo("UR initialized, sleeping 2s")
        rospy.set_param("/ur_initialized", True)
        
    else:
        # MoveIt to Pose not working
        switch_to_moveit()
        rospy.loginfo("Using moveit")
        # pose_goal = Pose(position=Point(*ur.first_pose_rel[:3]), orientation=Quaternion(*ur.first_pose_rel[3:]))
        pose_goal = ur.first_pose_rel
        rospy.loginfo(f"Pose for MoveIT: {pose_goal}")
        # pose_ur_map = ur.path.poses[0].pose
        # pose_goal = pose_ur_map

        # ur.group.set_pose_reference_frame("map")
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "base_footprint"
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose = pose_goal
        ur.group.set_pose_target(pose_msg)
        success = ur.group.go(wait=True)
        rospy.loginfo(f"Moveit success: {success}")
        if success is not True:
            rospy.logerr(f"Moveit failed pose_goal: {pose_goal}")
            sys.exit(1)
        switch_to_velocity()
        rospy.set_param("/ur_initialized", True)
    

        