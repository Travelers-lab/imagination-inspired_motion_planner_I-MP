from datetime import datetime
import pybullet as p
import time
import pybullet_data
import math
import numpy as np
from os.path import join, dirname,abspath
from robotEnvironment.robotEnvironmentConfig import Robot
#接物理引擎
physicsCilent = p.connect(p.GUI)

#设定机器人的目标位置和关机属性值
joint_target_P = 1
max_force = 100
target_V = 0.01
# rp = [joint_target_P for _ in range(6)]
#添加资源路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#显示资源和重力设置
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
#p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
#p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.setGravity(0, 0, -10)
t = 0.

#加载地面的URDF及初始条件设置
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 0.2]

model_dir = join(dirname(abspath(__file__)), 'envDescription/single_arm/left_arm.urdf')

startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robot = Robot(bullet_client=p, path=model_dir, obj_num=2, fixed_prob=0.5)
#tableID = p.loadURDF("simulation_table/objects.urdf", startPos, startOrientation)

#关节信息
# joint_num = p.getNumJoints(boxId)
# print(joint_num)
# available_joint_indexes = [i for i in range(joint_num) if p.getJointInfo(boxId, i)[2] != p.JOINT_FIXED]
# print(available_joint_indexes)
#for i in range(joint_num):




    #
#臂起始姿态
# rp = [3.14159, 0.0, 0.0, -1.5708, 0, 0.]
# for joint_Index in range(len(available_joint_indexes)):
#     p.resetJointState(boxId, available_joint_indexes[joint_Index], rp[joint_Index])
# #打印机械臂基关节和末端关节的位置信息
# for i in [0, 5]:
#     base_joint_states = p.getLinkState(bodyUniqueId=boxId, linkIndex=available_joint_indexes[i])
#     print('base_link.pose:{}'.format(base_joint_states[4]))


useSimulation = 0
useRealTimeSimulation = 0
p.setRealTimeSimulation(useRealTimeSimulation)
ikSolver = 0
# , 1.1434507369995117
target = [0.75, 0.35, 1.1434507369995117]

while True:
    agent_pos = robot.get_effector_states()
    print('agent_pos:', agent_pos['pos'].vec_p)
    joint_pos = robot.solving_inverse_problem(target)
    print(f'joint_pos:{joint_pos}')

    p.stepSimulation()
    time.sleep(1./240)

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
#print(cubePos, cubeOrn)
p.disconnect()