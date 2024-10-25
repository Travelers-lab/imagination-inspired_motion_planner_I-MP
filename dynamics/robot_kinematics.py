import math
import numpy as np
from numpy.linalg import pinv

def form_urdf_to_hd(urdf_param):
    x = urdf_param[0]
    y = urdf_param[1]
    z = urdf_param[2]
    rx = urdf_param[3]
    ry = urdf_param[4]
    rz = urdf_param[5]
    theta = rz
    alpha = math.atan2(y, x) * (180/math.π)
    d = z / math.cos(alpha)
    a = d * math.tan(alpha)
    dh_param = [d, theta, a, alpha]
    return dh_param

def from_dh_to_urdf(dh_params):
    d = dh_params[0]
    theta = dh_params[1]
    a = dh_params[2]
    alpha = dh_params[3]
    x = a * math.cos(theta)
    y = a * math.sin(theta)
    z = d
    rx = - alpha * math.cos(theta) - d * math.sin(theta) / a
    ry = alpha * math.sin(theta) - d * math.cos(theta) / a
    rz = theta
    return [x, y, z, rx, ry, rz]

###   DH_params:  [d, theta, a, alpha]
# DH_params = [[1.2465, 3.14156, -0.262, 1.5708], [0.36685, 0, 0, 0], [0, 0, 0, 1.5708], [0, 0, -0.24335, 0], [0, 0, -0.2132, 0], [0.13105, 0, 0 ,1.5708], [0.08355, 0, 0 , -1.5708], [0.0921, 0, 0, 0]]

class RobotKinematics:

    def __init__(self, DH_params):
        self.DH_params = DH_params

    def trans_matrix(self, joint_id, joint_pos):
        T = np.zeros((4, 4))
        # print("joint_pos:{},DH_params[1]:{} ".format(joint_pos, self.DH_params[joint_id][1]))
        ###   DH_params:  [d, theta, a, alpha]
        theta = self.DH_params[joint_id][1] + joint_pos
        c_theta = math.cos(theta)
        s_theta = math.sin(theta)
        c_alpha = math.cos(self.DH_params[joint_id][3])
        s_alpha = math.sin(self.DH_params[joint_id][3])
        T[0, 0] = c_theta
        T[0, 1] = -s_theta
        T[0, 3] = self.DH_params[joint_id][2]
        T[1, 0] = s_theta * c_alpha
        T[1, 1] = c_theta * c_alpha
        T[1, 2] = -s_alpha
        T[1, 3] = -self.DH_params[joint_id][0] * s_alpha
        T[2, 0] = s_theta * s_alpha
        T[2, 1] = c_theta * s_alpha
        T[2, 2] = c_alpha
        T[2, 3] = self.DH_params[joint_id][0] * c_alpha
        T[3, 3] = 1
        # print('T:{}'.format(T))
        return T

    def frame_position_and_rotation(self, joint_id, joint_pos):
        T = self.frame_trans_matrix(joint_id, joint_pos)
        # print("T[:3, 3]:{}, T[:3, :3]:{}".format(T[:3, 3], T[:3, :3]))
        return T[:3, 3], T[:3, :3], T

    def frame_trans_matrix(self, joint_id, joint_pos):
        # print("joint_id:{}, joint_pos:{}".format(joint_id, joint_pos))
        joint_pos = np.hstack((np.array([0]), joint_pos))

        T = np.eye(4)
        for i in range(joint_id + 1):
            # print("joint_id:{}".format(i))
            T_i = self.trans_matrix(i, joint_pos[i])
            T = T.dot(T_i)
        return T

    def jacobian(self, joint_id, joint_pos):
        J = []
        ##获得末端关节位姿
        effector_position, effector_rotation, effector_trans_matrix = self.frame_position_and_rotation(joint_id=joint_id, joint_pos=joint_pos)
        for i in range(2):
            ###获得每一个关节的位置和姿态
            frame_pos, frame_ori, frame_trans_matrix = self.frame_position_and_rotation(joint_id=i+2, joint_pos=joint_pos[:i+3])
            ###获得第三列
            z = frame_ori[:3, 2]
            p = np.array(effector_position) - np.array(frame_pos)
            p_c = np.cross(z, p)
            print("p:{},Z:{} p_c:{}".format(p, z, p_c))
            # for j in range(len(p_c)):
            #     if j == 2:
            #         p_c[j] = 1
            J.append(p_c)
        return np.array(J).T



def main():
    # DH_params = [[1.2465, 3.14159, -0.262, -1.5708], [0.36685, -0.7854, 0, 0], [0, 0, 0, 1.5708], [0, 0, -0.24335, 0],
    #              [0, 0, -0.2132, 0], [0.13105, 0, 0, 1.5708], [0.08355, 0, 0, -1.5708], [0.0921, 0, 0, 0]]
    ###   DH_params:  [d, theta, a, alpha]2.35619
    DH_params = [[1.2465, 0, 0.262, 0], [0.36685, 0., 0.0, -1.5708], [0, 0, 0, -1.5708], [0, 0, -0.24335, 0],
                 [0.13105, 0, -0.2132, 0], [0.08535, 0, 0, 1.5708], [0.0921, 0, 0, -1.5708]]
    robot = RobotKinematics(DH_params)
    joint_pos = [0.0, 0, 0, 0, 0, 0]
    position, rotation, matrix = robot.frame_position_and_rotation(6, joint_pos)
    print("position:{}, rotation:{}, matrix:{}".format(position, rotation, matrix))

    # jacobian = robot.jacobian(joint_id=1, joint_pos=joint_pos)
    # j_1 = pinv(jacobian)
    # print("jacobian:\n {}, \n j_1:\n {}".format(jacobian, j_1))

if __name__ == "__main__":
    main()
