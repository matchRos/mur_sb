#! /usr/bin/env python3

from math import atan2, sqrt, cos, sin
from mypath import Mypath
import rospy
from nav_msgs.msg import Path
from scipy.signal import savgol_filter
from matplotlib import pyplot as plt
from geometry_msgs.msg import PoseStamped, Pose

from match_lib.match_robots import MirNav2Goal

class trajectory_generation():



    def config(self):
        self.mir_target_vel_lin = 0.05
        self.ur_target_vel_lin = 0.05
        self.control_rate = 100
        self.ur_acc_limit_lin = 0.3
        self.mir_acc_limit_lin = 0.3
        self.w_limit = 1.0  # does nothing


    def __init__(self):
        rospy.init_node("trajectory_generation_node")
        rospy.Subscriber("mir_path", Path, self.mir_path_cb)
        rospy.Subscriber("ur_path", Path, self.ur_path_cb)
        self.config()
        
        self.target_reached = False
        self.mir_path = Mypath()
        self.mir_path.x = []
        self.mir_path.y = []
        self.mir_target_path = Mypath()
        self.mir_target_path.x = []
        self.mir_target_path.y = []
        self.mir_path.phi = []
        self.mir_path.v = [0.0]
        self.mir_path.w = [0.0]
        self.ur_path = Mypath()
        self.ur_path.x = []
        self.ur_path.y = []
        self.ur_path.z = []
        self.ur_target_path = Mypath()
        self.ur_target_path.x = []
        self.ur_target_path.y = []
        self.ur_target_path.z = []
        self.ur_path.phi = []
        self.ur_path.v = [0.0]
        self.ur_path.w = [0.0]

        rospy.spin()

    def run(self):
        ######## MiR ################
        path_len = len(self.raw_mir_path.poses)
        for i in range(0,path_len):
            self.mir_path.x.append(self.raw_mir_path.poses[i].pose.position.x)
            self.mir_path.y.append(self.raw_mir_path.poses[i].pose.position.y)

        
        for i in range(0,path_len-1):
            phi = atan2(self.mir_path.y[i+1]-self.mir_path.y[i],self.mir_path.x[i+1]-self.mir_path.x[i])
            self.mir_path.phi.append(phi)

        for i in range(0,path_len-2):
            self.mir_path.v.append(sqrt((self.mir_path.x[i+1]-self.mir_path.x[i])**2 + (self.mir_path.y[i+1]-self.mir_path.y[i])**2 ))
            self.mir_path.w.append(self.mir_path.phi[i+1]-self.mir_path.phi[i])

        ##################### UR ###################

        path_len = len(self.raw_ur_path.poses)
        for i in range(0,path_len):
            self.ur_path.x.append(self.raw_ur_path.poses[i].pose.position.x)
            self.ur_path.y.append(self.raw_ur_path.poses[i].pose.position.y)
            self.ur_path.z.append(self.raw_ur_path.poses[i].pose.position.z)

        
        for i in range(0,path_len-1):
            phi = atan2(self.ur_path.y[i+1]-self.ur_path.y[i],self.ur_path.x[i+1]-self.ur_path.x[i])
            self.ur_path.phi.append(phi)

        for i in range(0,path_len-2):
            self.ur_path.v.append(sqrt((self.ur_path.x[i+1]-self.ur_path.x[i])**2 + (self.ur_path.y[i+1]-self.ur_path.y[i])**2 + (self.ur_path.z[i+1]-self.ur_path.z[i])**2 ))
            self.ur_path.w.append(self.ur_path.phi[i+1]-self.ur_path.phi[i])
        
        #print(self.mir_path.v)
        self.compute_trajectory()



    def compute_trajectory(self):
        
        mir_index = 1
        ur_index = 1
        current_velocity_lin = 0.0
        mir_current_velocity_lin = 0.0
        ur_current_velocity_lin = 0.0
        mir_dist = 0.0
        ur_dist = 0.0
        mir_target_pose = Mypath()
        mir_target_pose.x = self.mir_path.x[0]
        mir_target_pose.y = self.mir_path.y[0]
        ur_target_pose = Mypath()
        ur_target_pose.x = self.ur_path.x[0]
        ur_target_pose.y = self.ur_path.y[0]
        ur_target_pose.z = self.ur_path.z[0]
        mir_path_distance = self.mir_path.v[0]
        mir_current_angle = self.mir_path.phi[0]
        ur_path_distance = self.ur_path.v[0]
        ur_current_angle = self.ur_path.phi[0]

        while not rospy.is_shutdown() and self.target_reached == False:
            # calculate remaining distance to the next control point
            #mir_dist_to_cp = sqrt((self.mir_path.y[index]-mir_target_pose.y)**2 + (self.mir_path.x[index]-mir_target_pose.x)**2)


            mir_acc_lin = self.mir_target_vel_lin / self.control_rate - mir_current_velocity_lin
            ur_acc_lin = self.ur_target_vel_lin / self.control_rate - ur_current_velocity_lin

            # limit acceleration
            if abs(mir_acc_lin) > self.mir_acc_limit_lin:
                mir_acc_lin *= (abs(mir_acc_lin) / mir_acc_lin) * self.mir_acc_limit_lin

            if abs(ur_acc_lin) > self.ur_acc_limit_lin:
                ur_acc_lin *= (abs(ur_acc_lin) / ur_acc_lin) * self.ur_acc_limit_lin

            mir_current_velocity_lin += mir_acc_lin/self.control_rate
            ur_current_velocity_lin += ur_acc_lin/self.control_rate

            if mir_dist + mir_current_velocity_lin >= mir_path_distance:
                while mir_dist + mir_current_velocity_lin >= mir_path_distance and not rospy.is_shutdown():
                    mir_index += 1
                    if mir_index > len(self.mir_path.v)-2:
                        self.target_reached = True
                        print("target reached")
                        break
                    else:
                        mir_path_distance += self.mir_path.v[mir_index]

            if ur_dist + ur_current_velocity_lin >= ur_path_distance:
                while ur_dist + ur_current_velocity_lin >= ur_path_distance and not rospy.is_shutdown():
                    ur_index += 1
                    if ur_index > len(self.ur_path.v)-2:
                        self.target_reached = True
                        print("target reached")
                        break
                    else:
                        ur_path_distance += self.ur_path.v[ur_index]


            mir_dist_to_cp = sqrt((self.mir_path.y[mir_index]-mir_target_pose.y)**2 + (self.mir_path.x[mir_index]-mir_target_pose.x)**2)
            if mir_dist_to_cp<mir_current_velocity_lin:
                mir_index += 1
            if mir_index >= len(self.mir_path.y):
                break

            ur_dist_to_cp = sqrt((self.ur_path.y[ur_index]-ur_target_pose.y)**2 + (self.ur_path.x[ur_index]-ur_target_pose.x)**2 + (self.ur_path.z[ur_index]-ur_target_pose.z)**2)
            if ur_dist_to_cp<ur_current_velocity_lin:
                ur_index += 1
            if ur_index >= len(self.ur_path.y):
                break


            mir_target_angle = atan2(self.mir_path.y[mir_index]-mir_target_pose.y, self.mir_path.x[mir_index]-mir_target_pose.x)
            ur_target_angle_horizontal = atan2(self.ur_path.y[ur_index]-ur_target_pose.y, self.ur_path.x[ur_index]-ur_target_pose.x)
            ur_target_angle_vertical = atan2(self.ur_path.z[ur_index]-ur_target_pose.z, sqrt((self.ur_path.y[ur_index]-ur_target_pose.y)**2 + (self.ur_path.x[ur_index]-ur_target_pose.x)**2))

            #print(ur_target_angle_vertical,ur_target_angle_horizontal)

            mir_w_target=mir_target_angle-mir_current_angle

            if abs(mir_w_target) > self.w_limit:
                mir_w_target *= (abs(mir_w_target) / mir_w_target) * abs(self.w_limit)
            
            mir_current_angle += mir_w_target

            mir_target_pose.x = mir_target_pose.x + cos(mir_target_angle) * mir_current_velocity_lin
            mir_target_pose.y = mir_target_pose.y + sin(mir_target_angle) * mir_current_velocity_lin

            ur_target_pose.x = ur_target_pose.x + cos(ur_target_angle_horizontal) * ur_current_velocity_lin * cos(ur_target_angle_vertical)
            ur_target_pose.y = ur_target_pose.y + sin(ur_target_angle_horizontal) * ur_current_velocity_lin * cos(ur_target_angle_vertical)
            ur_target_pose.z = ur_target_pose.z + ur_current_velocity_lin * sin(ur_target_angle_vertical)


            mir_dist += mir_current_velocity_lin / self.control_rate
            ur_dist += ur_current_velocity_lin / self.control_rate
            self.mir_target_path.x.append(mir_target_pose.x)
            self.mir_target_path.y.append(mir_target_pose.y)
            self.ur_target_path.x.append(ur_target_pose.x)
            self.ur_target_path.y.append(ur_target_pose.y)
            self.ur_target_path.z.append(ur_target_pose.z)


        self.mir_xhat = savgol_filter(self.mir_target_path.x, 51, 3) # window size 51, polynomial order 3
        self.mir_yhat = savgol_filter(self.mir_target_path.y, 51, 3) # window size 51, polynomial order 3
        self.ur_xhat = savgol_filter(self.ur_target_path.x, 51, 3) # window size 51, polynomial order 3
        self.ur_yhat = savgol_filter(self.ur_target_path.y, 51, 3) # window size 51, polynomial order 3
        self.ur_zhat = savgol_filter(self.ur_target_path.z, 51, 3) # window size 51, polynomial order 3

        self.trajectory_publisher()

       ### Plot ####
        
        rospy.loginfo("trajecotry computation complete")

        # f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
        # ax2.plot(self.mir_path.x,self.mir_path.y)
        # ax1.plot(self.ur_xhat,self.ur_yhat)
        # plt.show()

        mir_v = [0.0]
        mir_w = [atan2(self.mir_yhat[1]-self.mir_yhat[0], self.mir_xhat[1]-self.mir_xhat[0])]
        
        ur_v = [0.0]
        ur_phi = [atan2(self.mir_yhat[1]-self.mir_yhat[0], self.mir_xhat[1]-self.mir_xhat[0])]
        ur_w = [0.0]
        
        for i in range(1,len(self.mir_xhat)):
            ur_v.append(100*sqrt((self.ur_xhat[i]-self.ur_xhat[i-1])**2+(self.ur_yhat[i]-self.ur_yhat[i-1])**2))
            ur_phi.append(atan2(self.ur_yhat[i]-self.ur_yhat[i-1], self.ur_xhat[i]-self.ur_xhat[i-1]))
            
        for i in range(1,len(ur_phi)):
            ur_w.append((ur_phi[i]-ur_phi[i-1]))
    
        plt.figure()
        plt.plot(ur_v)
        plt.plot(ur_w)
        #plt.show()



    def trajectory_publisher(self):
        mir_pub = rospy.Publisher("mir_trajectory", Path, queue_size= 1)
        ur_pub = rospy.Publisher("ur_trajectory", Path, queue_size= 1)

        print("publishing trajectory to topic: " + mir_pub.resolved_name)

        mir_trajectory = Path()
        mir_trajectory_point = PoseStamped()
        mir_trajectory.header.frame_id = "map"
        mir_trajectory.header.stamp = rospy.Time.now()
        ur_trajectory = Path()
        ur_trajectory_point = PoseStamped()
        ur_trajectory.header.frame_id = "map"
        ur_trajectory.header.stamp = rospy.Time.now()

        # Because first goal of trajectory is not reached by cartesain formation controller if to far away:
        mir=MirNav2Goal("/mur216")
        first_pose = Pose()
        first_pose.position.x = self.mir_xhat[0]
        first_pose.position.y = self.mir_yhat[0]
        first_pose.orientation.w = 1.0
        mir.sendGoalPos(first_pose)
        rospy.logdebug("MiR at start position")

        mir_trajectory.poses = [PoseStamped() for i in range(len(self.mir_xhat))] 
        ur_trajectory.poses = [PoseStamped() for i in range(len(self.ur_xhat))]

        for i in range(0,len(self.mir_xhat)): #len(robot0_xhat)
            mir_trajectory_point.pose.position.x = self.mir_xhat[i]
            mir_trajectory_point.pose.position.y = self.mir_yhat[i]
            mir_trajectory.poses[i].pose.position.x = mir_trajectory_point.pose.position.x
            mir_trajectory.poses[i].pose.position.y = mir_trajectory_point.pose.position.y

        for i in range(0,len(self.ur_xhat)): #len(robot0_xhat)
            ur_trajectory_point.pose.position.x = self.ur_xhat[i]
            ur_trajectory_point.pose.position.y = self.ur_yhat[i]
            ur_trajectory_point.pose.position.z = self.ur_zhat[i]
            ur_trajectory.poses[i].pose.position.x = ur_trajectory_point.pose.position.x
            ur_trajectory.poses[i].pose.position.y = ur_trajectory_point.pose.position.y
            ur_trajectory.poses[i].pose.position.z = ur_trajectory_point.pose.position.z

        rospy.sleep(1)
        mir_pub.publish(mir_trajectory)
        rospy.sleep(2)
        ur_pub.publish(ur_trajectory)
        rospy.sleep(1)




    def mir_path_cb(self,Path):
        self.raw_mir_path = Path

    def ur_path_cb(self,Path):
        self.raw_ur_path = Path  
        self.run()
        


if __name__=="__main__":
    trajectory_generation()
    