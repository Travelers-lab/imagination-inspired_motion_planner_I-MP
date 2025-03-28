import sys
from os.path import join, dirname, abspath
path = join(dirname(dirname(abspath(__file__))))
sys.path.append(path)
import csv
import time
import math
import pybullet as p
import numpy as np
from omegaconf import OmegaConf
import pybullet_data as pd
from scipy.interpolate import interp1d
from robotEnvironment.robotEnvironment import Robot
from motionPlanner.vector import Vector
from motionPlanner.motionPlanning import MotionImagination

def set_simulation():
    clint_id = p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=90, cameraPitch=-89, cameraTargetPosition=[0.5, 0.3, 0])
    timeStep = 1 / 120
    p.setTimeStep(timeStep)
    p.setGravity(0, 0, -9.8)
    return timeStep, clint_id

def load_config():
    cfg = OmegaConf.load(join(dirname(dirname(abspath(__file__))), 'config/planningConfig.json'))
    return cfg

def write_data_to_csv(filename, joint_position):
    with open(filename, 'a', newline='') as file:
        writer = csv.writer(file)
        if file.tell() == 0:
            writer.writerow(["joint position"])
        writer.writerow([str(joint_position)])


def joint_command_generation(array):
    num_interp_points = 50
    interpolated_array = []
    for col in array.T:
        x = np.linspace(0, 1, len(col))  # 原始行索引
        f = interp1d(x, col, kind='linear')  # 创建线性插值函数
        x_new = np.linspace(0, 1, num_interp_points * array.shape[0])  # 新的行索引
        interpolated_array.append(f(x_new))

    interpolated_array = np.array(interpolated_array).T
    print(interpolated_array.shape)
    return interpolated_array


def set_environment(timestep, clint_id, cfg):
    flags = p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
    middle_command = np.array(
        [[-2.5, -1.4549361195299169, 1.5212827813087357, 0.00014673645429200314, 1.2567407279656975,
          -3.9682349577003087],
         [-1.5708, -1.4549361195299169, 1.5212827813087357, 0.00014673645429200314, 1.2567407279656975,
          -3.9682349577003087],
         [-0.7854, -0.2644589164266409, 1.2567298328063843, -2.5131488104648065, 1.521269294077356,
          0.0001081967702602979],
         [0.3490, -0.2644589164266409, 1.2567298328063843, -2.5131488104648065, 1.521269294077356,
          0.0001081967702602979],
         [0.8726, -0.9258413921372706, 0.9260884469121811, 8.500561123312466e-05, 1.5874368622192738,
          -6.2831007302365025],
         [2.5, -0.9258413921372706, 0.9260884469121811, 8.500561123312466e-05, 1.5874368622192738,
          -6.2831007302365025]])
    commands = joint_command_generation(middle_command)
    robot_path = join(dirname(dirname(abspath(__file__))), 'envDescription', 'single_arm', "right_arm.urdf")
    print(robot_path)

    robot = Robot(p, path=robot_path, clint_id=clint_id)
    # plant1 = p.loadURDF(fileName=join(dirname(dirname(abspath(__file__))), 'envDescription', 'simulation_table', 'table.urdf'), flags=flags,
    #                    basePosition=np.array([0.9, -0.2, 0.16]), useFixedBase=1)
    # plant2 = p.loadURDF(fileName=join(dirname(dirname(abspath(__file__))), 'envDescription', 'simulation_table', 'cylinder2.urdf'), flags=flags,
    #                    basePosition=np.array([0.55, -0.6, 1.18]), baseOrientation=[0, 0, 0, 1], useFixedBase=0)
    # plant3 = p.loadURDF(fileName='plane.urdf', flags=flags,
    #                     basePosition=np.array([0.0, 0.0, -0.07]), baseOrientation=[0, 0, 0, 1], useFixedBase=1)

    joint_pos = []
    for i in range(6):
        joint_pos.append(p.addUserDebugParameter(paramName=f'joint_{i}', rangeMin=-2*math.pi, rangeMax=2*math.pi, startValue=0))
    print(joint_pos)
    #mission 1
    data = np.array([[0.55, -0.6, 1.2425],
                     [0.55, -0.3, 1.2425],
                     [0.47, -0.3, 1.2425],
                     [0.47, -0.2, 1.2425],
                     [0.7, -0.2, 1.2425]])

    # mission 2
    data = np.array([[0.45, -0.5, 1.2425],[0.55, -0.5, 1.2425], [0.55, -0.15, 1.2425], [0.45, -0.15, 1.2425],
                    [0.45, -0.65, 1.2425], [0.55, -0.65, 1.2425], [0.55, -0.3, 1.2425],
                    [0.45, -0.3, 1.2425], [0.45, -0.15, 1.2425], [0.55, -0.15, 1.2425]])

    x = np.arange(data.shape[0])
    file_name = join(dirname(abspath(__file__)), 'joint_command.csv')
    x_new = np.linspace(0, data.shape[0] - 1, 5005)

    interpolated_data = np.zeros((5005, 3))
    for i in range(3):
        f = interp1d(x, data[:, i], kind='linear')
        interpolated_data[:, i] = f(x_new)

    for position in interpolated_data:
        # joint_command = []
        # for i in range(6):
        #     joint_command.append(p.readUserDebugParameter(int(joint_pos[i])))
        #
        # # joint_command = [8.611410386159863e-05, -0.2644589164266409, 1.2567298328063843, -2.5131488104648065, 1.521269294077356, 0.0001081967702602979]
        # joint_command[0] = p.readUserDebugParameter(int(joint_pos[0]))
        # robot.joint_position_controls_step(joint_command)
        # command = [0.6109042167663574, 0.10921423882246017, 0.5016182661056519]
        # robot.joint_position_controls_step(joint_command)
        # agent_state = robot.get_effector_states()

    # for command in range(commands.shape[0]):
    #     robot.joint_position_controls_step(commands[command])
    #     keys = p.getKeyboardEvents()
    #     enterKey = ord('q')
    #     if enterKey in keys and keys[enterKey] & p.KEY_WAS_TRIGGERED:
    #         joint_pos, joint_vel = robot.get_joint_states()
    #         e_skin_info = robot.get_eskin_states()
    #         effector_pos = robot.get_effector_states()
    #         write_data_to_csv(filename=join(dirname(abspath(__file__)), 'ur5_pos.csv'), joint_position=joint_pos, effector_pos=effector_pos, e_skin_information=e_skin_info)
    #     agent_state = robot.get_effector_states()
    #     if (target_point - agent_state['pos']).length <= 0.015:
    #         target_sequence += 1
    #         if target_sequence == len(target_points):
    #             print("complete the task!")
    #             break
    #         else:
    #             target_point = Vector(target_points[target_sequence])
    #     else:
    #         pass

        robot.cartesian_position_controls_step(position)
        agent_state = robot.get_effector_states()
        joint_position, joint_vel = robot.get_joint_states()
        write_data_to_csv(file_name, joint_position)

        print(agent_state['pos'].vec_p)
        p.stepSimulation()
        time.sleep(timestep)


def main():
    cfg = load_config()
    timestep, client_id = set_simulation()
    set_environment(timestep, client_id, cfg.imaged)


if __name__ == '__main__':
    main()