from os.path import dirname, join, abspath
import numpy as np
import pybullet_data
import random
import math

from dynamics.robot_kinematics import RobotKinematics
from motionPlanner.vector import Vector
from impedanceController.impedanceControl import CartesianImpedanceControl

class Robot:
    def __init__(self, bullet_client, path, clint_id):
        self.bullet_client = bullet_client
        self.description_file = join(dirname(dirname(abspath(__file__))), 'envDescription')
        self.table_file = join(dirname(dirname(abspath(__file__))), 'envDescription/simulation_table/table.urdf')
        self.bullet_client.setAdditionalSearchPath(pybullet_data.getDataPath())

        flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        self.robot = self.bullet_client.loadURDF(fileName=path,
                                                basePosition=np.array([0.0, 0.0, 0.0]),
                                                baseOrientation=[0, 0, 0, 1], flags=flags,
                                                useFixedBase=1, physicsClientId=clint_id)

        self.bullet_client.loadURDF("plane.urdf", physicsClientId=clint_id)

        self.bullet_client.loadURDF(fileName=self.table_file,
                                    basePosition=np.array([0.9, 0.3, 0.0]),
                                    baseOrientation=[0, 0, 0, 1], flags=flags,
                                    useFixedBase=1,
                                    physicsClientId=clint_id)

        self.available_joint_indexes = [i for i in range(self.bullet_client.getNumJoints(self.robot)) if
                                        self.bullet_client.getJointInfo(self.robot, i)[
                                            2] != self.bullet_client.JOINT_FIXED]
        self.fixed_joint_indexes = [i for i in range(self.bullet_client.getNumJoints(self.robot)) if
                                        self.bullet_client.getJointInfo(self.robot, i)[
                                            2] == self.bullet_client.JOINT_FIXED]

        print("available_joint:{}, \n fixed_joint:{}, \n joint num:{}, \n ".format(
            self.available_joint_indexes, self.fixed_joint_indexes, self.bullet_client.getNumJoints(self.robot)))
        for i in range(len(self.available_joint_indexes)):
            self.bullet_client.changeDynamics(bodyUniqueId=self.robot, linkIndex=self.available_joint_indexes[i],
                                              linearDamping=0.5, angularDamping=0.5)
            self.bullet_client.setJointMotorControl2(bodyIndex=self.robot,
                                                     jointIndex=self.available_joint_indexes[i],
                                                     controlMode=self.bullet_client.VELOCITY_CONTROL,
                                                     force=0)
        self.robotEndEffectorIndex = 11
        self.t = 0.
        DH_param = [[1.2465, 0, 0.262, 0], [0.36685, 0., 0.0, -1.5708], [0, 0, 0, 1.5708], [0, 0, -0.24335, 0],
                     [0.00945, 0, -0.2132, 0], [0.08535, 0, 0, 1.5708], [0.0, 0, 0, -1.5708], [0.0921, 0, 0, 0.0]]
        self.kinematics = RobotKinematics(DH_param)
        self.impedance_controller = CartesianImpedanceControl(kp=[5,5], kd=[1.5, 1.5])

        rp = np.array([-1.5708, 2.5, 0, 0, 0, 0.0, 0., 0.0, 0, 0, 0, 0, 0, 0])
        for joint_Index in range(len(self.available_joint_indexes)):
            self.bullet_client.resetJointState(self.robot, self.available_joint_indexes[joint_Index],
                                               rp[joint_Index])

        self.object = []

    def transformMatrix(self, body_links):
        body_trans = np.eye(4)
        trans_matrix = np.eye(4)
        trans_link = []
        sensor_transform_matrix = {}
        # sensor_position = {}
        # arm_radius = 0.042
        #计算传感器的基座的变换矩阵SE（3），大小为：4*4
        for i in body_links:
            joint_state = self.bullet_client.getLinkState(bodyUniqueId=self.robot, linkIndex=i)
            joint_pos, joint_ori = joint_state[4], joint_state[5]
            rot_matrix = self.bullet_client.getMatrixFromQuaternion(joint_ori)
            body_trans[:3, :3] = np.array(rot_matrix).reshape(3, 3)
            body_trans[:3, 3] = np.array(joint_pos)
            trans_link.append(body_trans.copy())
        #计算传感器的位置，以变换矩阵的形式。
        sensor_translation_param = [
                                    [0.0455, 0.0, 0.0],
                                    [0.0185, -0.0265, 0.0],
                                    [0.0185, -0.0275, 0.0],
                                    [-0.0125, 0.0]]
        sensor_rotation_matrix = {
                                  "body_link5sensor_line0": [1.701748, 2.6626, 3.62156, 4.58149],
                                  "body_link5sensor_line1": [1.22179, 2.18169, 3.14159, 4.10149, 5.06139],
                                  "body_link7sensor_line0": [1.701748, 2.6626, 3.62156, 4.58149],
                                  "body_link7sensor_line1": [1.22179, 2.18169, 3.14159, 4.10149, 5.06139],
                                  "body_link9sensor_line0": [3.272548, 4.2334, 5.19236, 6.15229],
                                  "body_link9sensor_line1": [2.79258, 3.75275, 4.71959, 5.67228, 6.63218],
                                  "body_link11sensor_line0": [0.5236, 1.5708, 2.6183, 3.66549, 4.71269, 5.759887],
                                  "body_link11sensor_line1": [0]}
        for i in range(len(body_links)):
            sensor_num = 0
            for j in range(len(sensor_translation_param[i])):
                if j != 2:
                    for k in sensor_rotation_matrix["body_link" + str(body_links[i]) + "sensor_line" + str(j)]:
                        first_line = [0.0, 0.0, sensor_translation_param[i][j]]
                        rot_matrix1 = self.bullet_client.getMatrixFromQuaternion([0.0, 0.0, math.sin(k / 2), math.cos(k / 2)])
                        trans_matrix[:3, :3] = np.array(rot_matrix1).reshape(3, 3)
                        trans_matrix[:3, 3] = np.array(first_line)
                        sensor_transform_matrix[
                            "body_link" + str(body_links[i]) + 'sensor_num' + str(sensor_num)] = np.dot(
                            trans_link[i], trans_matrix)
                        sensor_num += 1
                else:
                    first_line = [0.0, 0.0, sensor_translation_param[i][j]]
                    trans_matrix[:3, 3] = np.array(first_line)
                    sensor_transform_matrix[
                        "body_link" + str(body_links[i]) + 'sensor_num' + str(sensor_num)] = np.dot(
                        trans_link[i], trans_matrix)
                    # if i == 3 and j == 1:
                    #     point = [0, 0, sensor_translation_param[i][j], 1]
                    # else:
                    #     point = [arm_radius, 0, 0, 1]
                    # sensor_position[
                    #     'body_link' + str(body_links[i]) + 'sensor_num' + str(sensor_num)] = np.dot(
                    #     sensor_transform_matrix["joint" + str(body_links[i]) + 'sensor_num' + str(sensor_num)],
                    #     point)
                    sensor_num += 1
        return sensor_transform_matrix

    def get_effector_states(self):
        effector_info = self.bullet_client.getLinkState(self.robot,
                                                        self.robotEndEffectorIndex,
                                                        computeLinkVelocity=1)
        agent_state = {}
        agent_state['pos'] = Vector(list(effector_info[4][:2]))
        agent_state['vel'] = Vector(list(effector_info[6][:2]))
        return agent_state

    def load_environment(self, coordinate, fix_prob):
        # self.object.append(self.bullet_client.loadURDF(fileName=join(self.description_file, f'simulation_table/cylinder{1}.urdf'),
        #                             basePosition=np.array([0.5, 0.45, 1.01]),
        #                             baseOrientation=[0, 0, 0, 1],
        #                             flags=self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
        #                             useFixedBase=0))

        for i in range(len(coordinate)):
            fixed_base = 1 if random.random() < fix_prob else 0
            self.object.append(self.bullet_client.loadURDF(fileName=join(self.description_file, f'simulation_table/cylinder{i%6}.urdf'),
                                    basePosition=np.array([coordinate[i][0], coordinate[i][1], 1.01]),
                                    baseOrientation=[0, 0, 0, 1],
                                    flags=self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
                                    useFixedBase=fixed_base))

    def init_environment(self):
        rp = np.array([-1.5708, 2.5, 0, 0, 0, 0.0, 0., 0.0, 0, 0, 0, 0, 0, 0])
        for joint_Index in range(len(self.available_joint_indexes)):
            self.bullet_client.resetJointState(self.robot, self.available_joint_indexes[joint_Index],
                                               rp[joint_Index])
        for body_id in self.object:
            self.bullet_client.removeBody(body_id)
        self.object.clear()

    def _detect_collision_force(self):
        """
        Detect the collision force between the robot and each object in the simulation.
        Return False if the collision force is greater than 5N, otherwise return True.

        :return: A boolean indicating whether the collision force is within the limit.
        """
        contact_force = np.array([0, 0])
        for obstacle_id in self.object:
            # Get the contact points between the robot and the obstacle
            contact_points = self.bullet_client.getContactPoints(self.robot, obstacle_id)
            for contact in contact_points:
                # Extract the normal force from the contact point information
                contact_force = np.array(contact[7]) * contact[9]
                if contact[9] > 5:
                    return False, contact_force
        return True, contact_force

    def _detect_collision(self):
        """
        Detect whether the robot collides with any object in the simulation.
        Return False if a collision is detected, otherwise return True.

        :return: A boolean indicating whether the robot is collision-free.
        """
        for obstacle_id in self.object:
            # Get the contact points between the robot and the obstacle
            contact_points = self.bullet_client.getContactPoints(self.robot, obstacle_id)

            # If there are any contact points, a collision is detected
            if contact_points:
                return False

        # If no collisions are detected, return True
        return True

    def get_joint_states(self):
        joint_pose = [3.14159, 0.0, 0.0, -1.5708, 0, 0, 0]
        joint_vel = [0]*len(self.available_joint_indexes)
        for i in range(len(self.available_joint_indexes)):
            joint_state = self.bullet_client.getJointState(self.robot, self.available_joint_indexes[i])
            joint_pose[i+1] = joint_state[0]
            joint_vel[i] = joint_state[1]
        return joint_pose, joint_vel

    def _detect_local_minime(self, agent_path):
        agent_pos = self.get_effector_states()
        if len(agent_path) > 50:
            if agent_pos['vel'].length <1e-5:
                return False
            else:
                return True
        else:
            pass

    def calculate_jacobian(self):
        J = []
        endEffectorState = self.bullet_client.getLinkState(bodyUniqueId=self.robot,
                                                           linkIndex=self.robotEndEffectorIndex)
        endEffectorPos = endEffectorState[4]
        for i in range(len(self.available_joint_indexes)):
            joint_state = self.bullet_client.getLinkState(bodyUniqueId=self.robot,
                                                          linkIndex=self.available_joint_indexes[i])
            frame_pos, frame_ori = joint_state[4], joint_state[5]
            rot_matrix = self.bullet_client.getMatrixFromQuaternion(frame_ori)
            z_t = np.array(rot_matrix).reshape(3, 3)
            z = z_t[:3, 2]
            p = np.array(endEffectorPos) - np.array(frame_pos)
            p_c = np.cross(z, p)
            for j in range(len(p_c)):
                if j == 2:
                    p_c[j] = 1
            J.append(p_c)
        return np.array(J).T

    def cartesian_position_controls_step(self, pos):
        pos_control = pos
        # orn_control = self.bullet_client.getQuaternionFromEuler([0, 0, 0])
        # print(f'pos_control:{pos_control}')
        jointPoses = self.bullet_client.calculateInverseKinematics(self.robot, self.robotEndEffectorIndex,
                                                                   pos_control)
        for i in range(len(self.available_joint_indexes)):
            self.bullet_client.setJointMotorControl2(self.robot, self.available_joint_indexes[i],
                                                     self.bullet_client.POSITION_CONTROL, jointPoses[i])
        return self._detect_collision()

    def solving_inverse_problem(self, pos):
        joint_pose = self.bullet_client.calculateInverseKinematics(self.robot, self.robotEndEffectorIndex,
                                                                   pos)
        return joint_pose

    def joint_position_controls_step(self, jointPoses):
        for i in range(len(self.available_joint_indexes)):
            self.bullet_client.setJointMotorControl2(self.robot, self.available_joint_indexes[i],
                                                     self.bullet_client.POSITION_CONTROL, jointPoses[i],
                                                     targetVelocity=random.uniform(0.0001, 0.001))
        pass

    def torque_control_step(self, force, agent_path):
        force.append(0)
        joint_pos, joint_vel = self.get_joint_states()
        linear_jacobian = self.kinematics.jacobian(6, joint_pos)
        J = np.array(linear_jacobian)
        J = J.T
        tau_tal = J.dot(force)

        for i in range(len(self.available_joint_indexes)):
            self.bullet_client.setJointMotorControl2(bodyIndex=self.robot,
                                                     jointIndex=self.available_joint_indexes[i],
                                                     controlMode=self.bullet_client.TORQUE_CONTROL,
                                                     force=tau_tal[i])

        return self._detect_local_minime(agent_path)

    def velocity_control_steps(self, velocity):
        for i in range(len(self.available_joint_indexes)):
            self.bullet_client.setJointMotorControl2(bodyIndex=self.robot,
                                                     jointIndex=self.available_joint_indexes[i],
                                                     controlMode=self.bullet_client.VELOCITY_CONTROL,
                                                     targetVelocity=velocity[i],
                                                     force=50)

    def impedance_control_step(self, desire_pos, desire_vel, current_pos, current_vel, external_force, agent_path):
        virtual_force = self.impedance_controller.compute_virtual_force(desire_pos, current_pos[:2], desire_vel, current_vel, external_force)
        self.torque_control_step(virtual_force.tolist(), agent_path)
        safe_contact, contact_force = self._detect_collision_force()
        return safe_contact, contact_force

    def set_impedance_parameter(self, kd, kp):
        self.impedance_controller.kp = kp
        self.impedance_controller.kd = kd
        return