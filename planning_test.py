import time
import  argparse
import pybullet as p
import pybullet_data as pd
from omegaconf import OmegaConf
from os.path import join, dirname, abspath
from motionPlanner.vector import Vector
from motionPlanner.motionPlanning import MotionImagination
from robotEnvironment.robotEnvironmentConfig import Robot
from environmentUnderstanding.dataProcess import EnvironmentUnderstanding

timeStep = 1 / 120

def load_config():
    cfg = OmegaConf.load(join(dirname(abspath(__file__)), 'config/planningConfig.json'))
    return cfg

def set_environment():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(timeStep)
    robot_urdf = join(dirname(abspath(__file__)), 'envDescription/single_arm/left_arm.urdf')
    robot = Robot(bullet_client=p, path=robot_urdf, obj_num=2, fixed_prob=0.5)
    return robot

def planner_test(num_obj, fixed_prob):
    robot = set_environment()

    # environment info variables
    hsSensorInfo = {}
    objects = {}
    body_links = [11]

    # move mission
    target_points = [[0.55, 0.1], [0.75, 0.35], [0.55, 0.55]]
    point_sequence = 0
    target_point = Vector(target_points[point_sequence])
    t = 0

    # planning params
    f_active = Vector([0, 0])
    i = 0
    set_planning = True
    cfg = load_config()

    # set environment
    environment_understander = EnvironmentUnderstanding()
    motion_imager = MotionImagination(cfg=cfg)
    # robot.load_environment(num_obj=num_obj, fixed_prob=fixed_prob)

    while set_planning:
        agent_state = robot.get_effector_states()
        # print('pos:', agent_state['pos'].vec_p)
        if (target_point - agent_state['pos']).length <= 0.005:
            point_sequence += 1
            if point_sequence == len(target_points):
                print("the last target point!")
                set_planning = False
                # 重置环境，切换规划器，序列归零，传感器数据清空；
                continue
                # 保存数据：测试条件，路径长度，用时，成功与否；
            else:
                target_point = Vector(target_points[point_sequence])
        else:
            pass
        objects, hsSensorInfo = environment_understander.object_representation(robot.robot, robot.transformMatrix(body_links), hsSensorInfo,
                                            robot.object, objects, agent_state)
        f_total, t, objects, f_rep, f_att, f_interact = motion_imager.solving_gradient(objects=objects, f_active=f_active, agent_state=agent_state,
                                                                        target_point=target_point, t=t)
        print(f'f_total:{f_total}; f_rep:{f_rep.vec_p}; f_att:{f_att.vec_p};')
        if objects:
            print(objects['object3']['attribute'], objects['object3']['states'])
        robot.torque_control_step(f_total)
        p.stepSimulation()
        time.sleep(timeStep)
    p.disconnect()
   


if __name__ == "__main__":
    try:
        planner_test(num_obj=0, fixed_prob=0)
    except KeyboardInterrupt:
        pass