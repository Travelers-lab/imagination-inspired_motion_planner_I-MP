import numpy as np
import pybullet as p
from os.path import dirname, join, abspath
import pybullet_data


def jointInfo(bodyId, joint_index):
    joint_pos = np.zeros(len(joint_index))
    joint_vel = np.zeros(len(joint_index))
    joint_acc = np.zeros(len(joint_index))

    for i in range(len(joint_index)):
        #print("joint_acc:{}".format(data.ddq[num]))
        joint_info = p.getJointState(bodyId, joint_index[i])
        joint_pos[i] = joint_info[0]
        joint_vel[i] = joint_info[1]#joint_info[1]
        joint_acc[i] = 0.

    return joint_pos, joint_vel, joint_acc


