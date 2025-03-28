import numpy as np


class CartesianImpedanceControl:
    def __init__(self, kp, kd):
        """
        初始化笛卡尔空间阻抗控制器。
        :param kp: 刚度矩阵 (2x2 矩阵)。
        :param kd: 阻尼矩阵 (2x2 矩阵)。
        """
        self.kp = np.array(kp)  # 刚度矩阵
        self.kd = np.array(kd)  # 阻尼矩阵

    def compute_virtual_force(self, desired_pos, actual_pos, desired_vel, actual_vel, external_force):
        """
        计算虚拟力。
        :param desired_pos: 期望位置 (2x1 向量)。
        :param actual_pos: 实际位置 (2x1 向量)。
        :param desired_vel: 期望速度 (2x1 向量)。
        :param actual_vel: 实际速度 (2x1 向量)。
        :param external_force: 外部接触力反馈补偿 (2x1 向量)。
        :return: 虚拟力 (2x1 向量)。
        """
        pos_error = np.array(desired_pos) - np.array(actual_pos)

        vel_error = np.array(desired_vel) - np.array(actual_vel)

        virtual_force = self.kp * pos_error + self.kd * vel_error - external_force[:2]

        return virtual_force


# 示例使用
if __name__ == "__main__":
    # 定义刚度矩阵和阻尼矩阵
    kp = [[100, 0], [0, 100]]  # 刚度矩阵 (N/m)
    kd = [[10, 0], [0, 10]]  # 阻尼矩阵 (N·s/m)

    # 初始化阻抗控制器
    impedance_controller = CartesianImpedanceControl(kp, kd)

    # 定义期望位置、实际位置、期望速度、实际速度和外部力
    desired_pos = [1.0, 2.0]  # 期望位置 (m)
    actual_pos = [0.9, 1.8]  # 实际位置 (m)
    desired_vel = [0.1, 0.2]  # 期望速度 (m/s)
    actual_vel = [0.05, 0.15]  # 实际速度 (m/s)
    external_force = [0.0, 0.0]  # 外部力 (N)

    # 计算虚拟力
    virtual_force = impedance_controller.compute_virtual_force(
        desired_pos, actual_pos, desired_vel, actual_vel, external_force
    )

    # 输出结果
    print("虚拟力 (N):", virtual_force)
