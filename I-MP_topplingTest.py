import time
import csv
import json
import pybullet as p
import pybullet_data as pd
from omegaconf import OmegaConf
from os.path import join, dirname, abspath


from motionPlanner.vector import Vector
from robotEnvironment.robotEnvironmentKnockingOver import Robot
from compared_method.plannerPipeline import ImagedPipeline
from utils.utils import evaluate_update,check_time_difference, write_knocking_over_to_csv

timeStep = 1 / 240

def load_config():
    cfg = OmegaConf.load(join(dirname(abspath(__file__)), 'config/planningConfig.json'))
    return cfg

def set_environment():
    physicalID = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(timeStep)
    robot_urdf = join(dirname(abspath(__file__)), 'envDescription/single_arm/left_arm.urdf')
    robot = Robot(bullet_client=p, path=robot_urdf, clint_id=physicalID)
    return robot, physicalID


def variable_reset():
    environment_info = {"point_cloud_sets":[], "hs_sensor_info":{}, "objects":{}}
    task_progress = 0
    return environment_info, task_progress



def planner_test(cfg, thread):
    robot, client1 = set_environment()

    path_lengths = [0.313, 0.605, 0.855]
    target_points = [[0.75, 0.336]]
    target_sequence = 0

    # load planner
    imaged_planner = ImagedPipeline(cfg.imaged)

    # set simulation
    f_active = Vector([0, 0])
    t = 0
    lines = 0

    with open('./testCondition/stress_test_conditions.csv') as f:
        reader = csv.DictReader(f)
        for row in reader:
            lines += 1
            if lines  <= 20000:
                coordinates = json.loads(row['test_environments'])
                mission_to_be_completed = row['mission to be completed']
                object_num = int(row['num_obstacles'])
                fixed_prob = float(row["fixed_prob"])
                target_sequence = 0
                print("\n\ntask_num:", lines)
                local_time = time.time()
                formatted_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(local_time))
                print("current time: ", formatted_time)
                robot.init_environment()
                robot.load_environment(coordinates)
                environment_info, task_progress = variable_reset()
                target_point = Vector(target_points[target_sequence])
                set_planning = True
                while set_planning:
                    agent_state = robot.get_effector_states()
                    if (target_point - agent_state['pos']).length <= 0.005:
                        target_sequence += 1
                        print(f'target_sequence:{target_sequence}')
                        if target_sequence == len(target_points):
                            print("complete the task!")
                            task_progress = 1
                            write_knocking_over_to_csv(row['test_condition'], object_num, fixed_prob, task_progress, mission_to_be_completed, thread)
                            break
                        else:
                            target_point = Vector(target_points[target_sequence])
                    if check_time_difference(local_time) == False:
                        write_knocking_over_to_csv(row['test_condition'], object_num, fixed_prob, task_progress, mission_to_be_completed, thread)
                        break
                    actuation_force, t = imaged_planner.forward(robot, environment_info, agent_state, f_active,
                                                                target_point, t)
                    robot.torque_control_step(actuation_force)
                    p.stepSimulation(physicsClientId=client1)
                    time.sleep(timeStep)
                else:
                    p.disconnect(physicsClientId=client1)
        p.disconnect(physicsClientId=client1)
   


if __name__ == "__main__":
    try:
        thread = 0
        cfg = load_config()
        planner_test(cfg, thread)
    except KeyboardInterrupt:
        pass