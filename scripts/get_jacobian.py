#!/usr/bin/env python3

import rospy
import numpy as np

# from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64MultiArray

class SubscribeJacobian(object):

    def __init__(self,):
        rospy.Subscriber('/jacobian', Float64MultiArray, self.cb_jacobi)
        self.jm = np.zeros((6,6))

    def cb_jacobi(self, msg):  
        a = np.array(msg.data, dtype=np.float64)
    
        self.jm = a.reshape(6,6)

    def get_mat(self):
        return self.jm




if __name__ == "__main__":
    rospy.init_node("sub_jacobian_ur")
    jac = SubscribeJacobian()


    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rospy.loginfo(jac.get_mat())
        rate.sleep()

