#!/usr/bin/env python3

import rospy
import moveit_commander
import numpy as np

# from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout

class PubJacobian(object):

    def __init__(self, dim=(6,6)):
        """Jacobian Matrix is flattened and sent to topic /jacobian. joint states via '/joint_states'.
        Resize matrix with numpy.reshape(6,6) (dim from msg)

        Args:
            dim (tuple, optional): Dimension of JacobiMat. Defaults to (6,6).
        """
        self.layout = MultiArrayLayout()
        self.layout.dim.append(MultiArrayDimension())
        self.layout.dim[0].label = "0"
        self.layout.dim[0].size = dim[0]
        self.layout.dim.append(MultiArrayDimension())
        self.layout.dim[1].label = "1"
        self.layout.dim[1].size = dim[1]


        self.group = moveit_commander.MoveGroupCommander("manipulator")
        rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
        self.pub_jacobi = rospy.Publisher("/jacobian", Float64MultiArray, queue_size=10)

    def joint_states_cb(self, data):  
        a = data
        joint_states = [a.position[2], a.position[1], a.position[0], a.position[3], a.position[4], a.position[5]]   # only for UR?
        # ggf joint states via moveit

        jm = np.array(self.group.get_jacobian_matrix(joint_states), dtype=np.float64)
        jm_flattend = jm.reshape(-1)


        data_to_send = Float64MultiArray()
        data_to_send.data = jm_flattend
        data_to_send.layout = self.layout

        self.pub_jacobi.publish(data_to_send)



if __name__ == "__main__":
    rospy.init_node("pub_jacobian_ur")
    jac = PubJacobian()
    rospy.spin()

