import numpy as np
import math
from numpy.linalg import pinv
import pybullet as pb


def transformationMatrix(joint_param, pos):
    T = np.array([[math.cos(joint_param[2] + pos), -math.sin(joint_param[2] + pos), 0, joint_param[1]],
                  [math.sin(joint_param[2] + pos)*math.cos(joint_param[0]), math.cos(joint_param[2] + pos) * math.cos(joint_param[0]), -math.sin(joint_param[0]), -math.sin(joint_param[0]) * joint_param[3]],
                  [math.sin(joint_param[2] + pos)*math.sin(joint_param[0]), math.cos(joint_param[2] + pos) * math.sin(joint_param[0]), math.cos(joint_param[0]), math.cos(joint_param[0]) * joint_param[3]],
                  [0, 0, 0, 1]])
    return T

def calculateJacobain(endPose):
    joint_pose = np.zeros(3)
    for i in range(len(endPose)):
        joint_pose[i] = endPose[i]
    joint_param = [[0, 0, 3.14159, 0], [0, 0.24355, 0, 0], [0, 0.29328, 0, 0]]
    joint_trans = {}
    for i in range(len(joint_pose)):
        T = transformationMatrix(joint_param[i], joint_pose[i])
        joint_trans["frame"+str(i)] = T
    T23 = joint_trans["frame"+str(2)]
    omega23 = T23[:3, 2]
    vector23 = T23[:3, 3]
    p_c23 = np.cross(omega23, vector23)
    T13 = joint_trans["frame"+str(1)].dot(T23)
    omega13 = T13[:3, 2]
    vector13 = T13[:3, 3]
    p_c13 = np.cross(omega13, vector13)
    T03 = joint_trans["frame" + str(0)].dot(T13)
    omega03 = T03[:3, 2]
    vector03 = T03[:3, 3]
    p_c03 = np.cross(omega03, vector03)
    return np.array([p_c13, p_c23]).T


if __name__ == "__main__":
    pose = [1.5708, 1.5783]
    j = calculateJacobain(pose)
    print(j)
    j_1 = pinv(j)
    print(j_1)





