#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

def talker():
    pub = rospy.Publisher('wall_frame_base',  PoseWithCovarianceStamped, queue_size=10)
    rospy.init_node('pub_wall_pose', anonymous=True)
    
    pose_wall = PoseWithCovarianceStamped()
    pose_wall.pose.pose.position.x = 4.8
    pose_wall.pose.pose.position.y = 10.0
    
    pose_wall.pose.pose.orientation.z = 0.687
    pose_wall.pose.pose.orientation.w = 0.725
    pub.publish(pose_wall)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass