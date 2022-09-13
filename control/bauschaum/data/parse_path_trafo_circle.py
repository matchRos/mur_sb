#! /usr/bin/env python3

# NOT WORKING BECAUSE OF DIFFEERENT LENGTHS AND ENDPoints OF THE PATHS!

import rospy
import tf
from tf import transformations
from circle_mirx import circle_mirx as mir_x
from circle_miry import circle_miry as mir_y

from circle_urx import circle_urx as wall_x
from circle_ury import circle_ury as wall_y
from circle_urz import circle_urz as wall_z

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class parse_path_trafo():

    def __init__(self):
        rospy.init_node("parse_path_trafo_node")
        self.mir_x= mir_x()
        self.mir_y= mir_y()

        self.wall_x = wall_x()
        self.wall_y = wall_y()
        self.wall_z = wall_z()

        #print(mir_y)

        self.mir_pub = rospy.Publisher("mir_path", Path, queue_size= 1)
        self.ur_pub = rospy.Publisher("ur_path", Path, queue_size= 1)


        self.mir_path = Path()
        self.mir_path_point = PoseStamped()
        self.mir_path.header.frame_id = "map"
        self.mir_path.header.stamp = rospy.Time.now()
        self.ur_path = Path()
        self.ur_path_point = PoseStamped()
        self.ur_path.header.frame_id = "map"
        self.ur_path.header.stamp = rospy.Time.now()

        rospy.Subscriber("/wall_frame_base", PoseWithCovarianceStamped, self.wall_frame_cb)
        rospy.spin()



    def wall_frame_cb(self,PoseWithCovarianceStamped):
        self.x = PoseWithCovarianceStamped.pose.pose.position.x
        self.y = PoseWithCovarianceStamped.pose.pose.position.y
        self.z = PoseWithCovarianceStamped.pose.pose.position.z
        q1 = PoseWithCovarianceStamped.pose.pose.orientation.x
        q2 = PoseWithCovarianceStamped.pose.pose.orientation.y
        q3 = PoseWithCovarianceStamped.pose.pose.orientation.z
        q4 = PoseWithCovarianceStamped.pose.pose.orientation.w
        self.R = transformations.quaternion_matrix([q1,q2,q3,q4])

        self.run()


    def run(self):

        self.mir_path.poses = [PoseStamped() for i in range(len(self.mir_x))] 
        self.ur_path.poses = [PoseStamped() for i in range(len(self.wall_x))] 

        mir_path_shift = 0 #50
        ur_path_shift = mir_path_shift-0 #shift correction from mir_path to ur_path
        for i in range(0,len(self.mir_x)-mir_path_shift): #len(robot0_xhat) 
            self.mir_path_point.pose.position.x = self.x + self.mir_x[i+mir_path_shift] * self.R[0][0] + self.mir_y[i+mir_path_shift] * self.R[0][1]
            self.mir_path_point.pose.position.y = self.y + self.mir_x[i+mir_path_shift] * self.R[1][0] + self.mir_y[i+mir_path_shift] * self.R[1][1]

            self.mir_path.poses[i].pose.position.x = self.mir_path_point.pose.position.x
            self.mir_path.poses[i].pose.position.y = self.mir_path_point.pose.position.y

        for i in range(0,len(self.wall_x)-mir_path_shift): #len(robot0_xhat)
            self.ur_path_point.pose.position.x = self.x + self.wall_x[i+ur_path_shift] * self.R[0][0] + self.wall_y[i+ur_path_shift] * self.R[0][1]
            self.ur_path_point.pose.position.y = self.y + self.wall_x[i+ur_path_shift] * self.R[1][0] + self.wall_y[i+ur_path_shift] * self.R[1][1]
            self.ur_path_point.pose.position.z = self.z + self.wall_z[i+ur_path_shift]

            self.ur_path.poses[i].pose.position.x = self.ur_path_point.pose.position.x
            self.ur_path.poses[i].pose.position.y = self.ur_path_point.pose.position.y
            self.ur_path.poses[i].pose.position.z = self.ur_path_point.pose.position.z

        
        self.mir_pub.publish(self.mir_path)
        rospy.sleep(0.01)
        self.ur_pub.publish(self.ur_path)
        
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #print(self.mir_path.poses[0].pose.position.x, self.mir_path.poses[0].pose.position.y)
            br_ur = tf.TransformBroadcaster()
            br_ur.sendTransform((self.ur_path.poses[0].pose.position.x, self.ur_path.poses[0].pose.position.y, self.ur_path.poses[0].pose.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "start_pos_ur",
                        "map")
            br_mir = tf.TransformBroadcaster()
            br_mir.sendTransform((self.mir_path.poses[0].pose.position.x, self.mir_path.poses[0].pose.position.y, 0),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "start_pos_mir",
                        "map")
            rate.sleep()



if __name__=="__main__":
    parse_path_trafo()
    
