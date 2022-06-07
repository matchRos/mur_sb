#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

def talker():
    pub = rospy.Publisher('wall_frame_base',  PoseStamped, queue_size=10)
    rospy.init_node('pub_wall_pose', anonymous=True)
    
    pose_wall = PoseStamped()
    pose_wall.pose.position.x = 4.8
    pose_wall.pose.position.y = 10.0
    
    pose_wall.pose.orientation.z = 0.687
    pose_wall.pose.orientation.w = 0.725
    pub.publish(pose_wall)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass