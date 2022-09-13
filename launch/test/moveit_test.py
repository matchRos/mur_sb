from turtle import position
import rospy
import sys
import moveit_commander
from geometry_msgs.msg import PoseStamped,Pose,Point,Quaternion

group_name = "UR_arm"
ns="mur216"
rospy.init_node("moveit_test_node")
moveit_commander.roscpp_initialize(sys.argv)
group = moveit_commander.MoveGroupCommander(group_name, robot_description= ns+"/robot_description", ns=ns,wait_for_servers=5.0) #, ns=ns)

p_list=[1.0539353097548936, 0.3902559230154179, 0.4139905174799845, -0.001001108467051236, 0.7071061390032947, 0.7071054543975931, 0.0013350381506873012]
# p_list = [0.2,0.2,0.0,0.0,0.0,0.0,1.0]
goal_p= PoseStamped()
goal_p.header.frame_id = "base_footprint"
goal_p.header.stamp = rospy.Time.now()
goal_p.pose = Pose(position=Point(*p_list[:3]), orientation=Quaternion(*p_list[3:]))
group.set_pose_target(goal_p)
success = group.go(wait=True)
rospy.loginfo(f"Moveit success: {success}")
if success is not True:
    rospy.logerr(f"Moveit failed pose_goal: {goal_p}")

ee_link = group.get_end_effector_link()
target_pose = group.get_current_pose(ee_link)
print(ee_link)
print(target_pose)