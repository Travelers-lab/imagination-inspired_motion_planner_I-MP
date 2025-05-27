import time
import random
import argparse
import numpy as np
import pybullet as p
import pybullet_data as pd
import multiprocessing as mp
from omegaconf import OmegaConf
from os.path import join, dirname, abspath
from motionPlanner.vector import Vector

from motionPlanner.motionPlanning import MotionImagination
from robotEnvironment.robotEnvironmentConfig import Robot
from testCondition.testScenarioGenerate import generate_obstacle_coords
from environmentUnderstanding.approachingPerception import tactile_collection
from environmentUnderstanding.dataProcess import EnvironmentUnderstanding
from utils.utils import write_data_to_csv, evaluate_update, check_time_difference
from compared_method.plannerPipeline import ModeledPipeline, SimulatedPipeline, SampledPipeline, ImagedPipeline
from compared_method.simulation_basedPlanner.externalSimulator import PathExecutionValidator
timeStep = 1 / 240

def load_config():
    cfg = OmegaConf.load(join(dirname(abspath(__file__)), 'config/planningConfig.json'))
    return cfg

def add_arg():
    parser = argparse.ArgumentParser(description="A set of args for testing motion planners.")
    parser.add_argument("-t","--test_num", type=int, required=True, help="Number of tests")
    parser.add_argument("-o", "--object_num", type=int, required=True, help="Number of objects")
    parser.add_argument("-f", "--fixed_probability", type=float, required=True, help="Obstacles are set with a fixed probability")
    args = parser.parse_args()
    return args


def set_environment(coords, fixed_prob):
    client1 = p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.setPhysicsEngineParameter(numSolverIterations=50)
    p.setGravity(0, 0, -9.8, physicsClientId=client1)
    p.setTimeStep(timeStep)
    robot_urdf = join(dirname(abspath(__file__)), 'envDescription/single_arm/left_arm.urdf')
    robot = Robot(bullet_client=p, path=robot_urdf, clint_id=client1)
    robot.load_environment(coordinate=coords, fix_prob=0)
    return robot, client1

def variable_reset():
    environment_info = {"point_cloud_sets":[], "hs_sensor_info":{}, "objects":{}}
    evaluate_metrics = {"time_cost":0.0, "path_cost":0, "task_progress":0}
    target_sequence = 0
    agent_path = []
    return environment_info, evaluate_metrics, target_sequence, agent_path

def simulator_2(cfg, queue_traj, queue_result, queue_obstacles, queue_mission):
    set_simulation = True
    path_validator = PathExecutionValidator(cfg.simulated.robot_urdf_path)
    reset_flag = False
    while set_simulation:
        trajectory = queue_traj.get()
        obstacle_coordinates = queue_obstacles.get()
        try:
            reset_flag = queue_mission.get(block=False)
        except mp.queues.Empty:
            pass
        path = path_validator.validate_path(obstacle_coordinates, trajectory)
        queue_result.put(path)
        if reset_flag == True:
            path_validator.reset_environment()
            reset_flag = False
        time.sleep(0.01)
    p.disconnect(physicsClientId=path_validator.physicsClient)



def simulator_1(queue_traj, queue_result, queue_mission, num_obj, fixed_prob, cfg):
    test_condition = []
    test_condition.append(f'object number: {str(num_obj)}')
    test_condition.append(f'fixation probability: {str(fixed_prob)}')

    # target_points = [[0.55, 0.1], [0.75, 0.35], [0.55, 0.55]]
    target_points = [[0.55, 0.1]]
    test_condition.append(f'task difficulty: {len(target_points)}')
    target_sequence = 0
    target_point = Vector(target_points[target_sequence])
    body_links = [11]

    f_active = Vector([0, 0])
    t = 0
    set_planning = True

    for i in range(20):

        # load environment
        robot, client1 = set_environment(num_obj, fixed_prob)
        # robot.load_environment(num_obj=num_obj, fixed_prob=fixed_prob)

        # motion task
        # target_points = [[0.58, 0.15], [0.75, 0.35], [0.55, 0.55]]

        # load_planner
        model_planner = ModeledPipeline(cfg.modeled)
        simulation_planner = SimulatedPipeline(cfg.simulated)
        sampled_planner = SampledPipeline(cfg.sampled)
        imaged_planner = ImagedPipeline(cfg.imaged)

        for planner in ['modeled', 'sampled', 'simulated', 'imagined']:
            if planner == 'simulated':
                robot.reset_environment()
                environment_info, evaluate_metrics, target_sequence, agent_path = variable_reset()
                set_simulation = True
                agent_state = robot.get_effector_states()
                start_point = agent_state['pos'].vec_p
                target_point = Vector(target_points[target_sequence])
                trajectory_idx = 0
                while set_simulation:
                    agent_state = robot.get_effector_states()
                    trajectory = simulation_planner.forward(environment_info, start_point[:2],
                                                            target_point.vec_p, queue_traj,
                                                            queue_result, queue_obstacles)
                    # print(f'trajectory{trajectory.shape}')
                    if trajectory.size == 0:
                        write_data_to_csv(test_condition, planner, evaluate_metrics)
                        queue_mission.put(True)
                        print('collision_false')
                        break
                    for i in range(trajectory_idx, trajectory_idx + 50):
                        safe_interact = robot.cartesian_position_controls_step(trajectory[i])
                        environment_info = tactile_collection(robot.transformMatrix(cfg.simulated.body_link), environment_info)
                        evaluate_metrics, agent_path = evaluate_update(evaluate_metrics, timeStep, agent_state,
                                                                           agent_path)
                        if safe_interact == False:
                            print(safe_interact)
                            write_data_to_csv(test_condition, planner, evaluate_metrics)
                            set_simulation = False
                            queue_mission.put(True)
                            print('collision_false')

                            break
                        else:
                            pass
                        agent_state = robot.get_effector_states()
                        # print('agent_pos:', agent_state['pos'].vec_p)
                        if (target_point - agent_state['pos']).length <= 0.012:
                            target_sequence += 1
                            print(f'target_sequence:{target_sequence}')
                            if target_sequence == len(target_points):
                                print("complete the task!")
                                set_simulation = False
                                evaluate_metrics['task_process'] = 1
                                write_data_to_csv(test_condition, planner, evaluate_metrics)
                                queue_mission.put(True)
                                break
                            else:
                                target_point = Vector(target_points[target_sequence])
                                start_point = agent_state['pos'].vec_p
                                print(f'start_point:{start_point}')
                                trajectory_idx = 0
                        else:
                            pass
                        p.stepSimulation(physicsClientId=client1)
                        time.sleep(timeStep)
                    if trajectory_idx + 50 <= 450:
                        trajectory_idx = trajectory_idx + 50
                    elif trajectory_idx + 50 >= 500:
                        trajectory_idx = 450
                    else:
                        trajectory_idx = 0
                    # print(f'trajectory_idx:{trajectory_idx}')

            if planner == 'imagined':
                robot.reset_environment()
                set_simulation = True
                environment_info, evaluate_metrics, target_sequence, agent_path = variable_reset()
                target_point = Vector(target_points[target_sequence])
                while set_simulation:
                    agent_state = robot.get_effector_states()
                    # print('agent_pos:', agent_state['pos'].vec_p)
                    if (target_point - agent_state['pos']).length <= 0.005:
                        target_sequence += 1
                        print(f'target_sequence:{target_sequence}')
                        if target_sequence == len(target_points):
                            print("complete the task!")
                            set_simulation = False
                            evaluate_metrics['task_process'] = 1
                            write_data_to_csv(test_condition, planner, evaluate_metrics)
                            queue_mission.put(True)
                            break
                        else:
                            target_point = Vector(target_points[target_sequence])
                    actuation_force = imaged_planner.forward(robot, environment_info, agent_state, f_active,
                                                            target_point, t)
                    safe_interact = robot.torque_control_step(actuation_force)
                    evaluate_metrics, agent_path = evaluate_update(evaluate_metrics, timeStep, agent_state,
                                                                       agent_path)
                    if safe_interact == False:
                        print(safe_interact)
                        write_data_to_csv(test_condition, planner, evaluate_metrics)
                        set_simulation = False
                        queue_mission.put(True)
                        print('collision_false')
                        break
                    else:
                        pass
                    p.stepSimulation(physicsClientId=client1)
                    time.sleep(timeStep)
            else:
                pass

        p.disconnect()


def planner_test(num_obj, fixed_prob):
    test_condition = []
    test_condition.append(f'object number: {str(num_obj)}')
    test_condition.append(f'fixation probability: {str(fixed_prob)}')

    # environment info variables
    hsSensorInfo = {}
    objects = {}
    body_links = [11]

    # motion task
    target_points = [[0.55, 0.1], [0.75, 0.35], [0.55, 0.55]]
    test_condition.append(f'task difficulty: {len(target_points)}')
    point_sequence = 0
    target_point = Vector(target_points[point_sequence])

    # load planner

    # planning params
    f_active = Vector([0, 0])
    i = 0
    set_planning = True
    cfg = load_config()

    # set environment
    environment_understander = EnvironmentUnderstanding()
    motion_imager = MotionImagination(cfg=cfg.imaged)
    # robot.load_environment(num_obj=num_obj, fixed_prob=fixed_prob)

    for i in range(20):

        # load environment
        robot, client1 = set_environment(num_obj, fixed_prob)
        # robot.load_environment(num_obj=num_obj, fixed_prob=fixed_prob)

        # motion task
        target_points = [[0.58, 0.15], [0.75, 0.35], [0.55, 0.55]]

        # load_planner
        model_planner = ModeledPipeline(cfg.modeled)
        simulation_planner = SimulatedPipeline(cfg.simulated)
        sampled_planner = SampledPipeline(cfg.sampled)
        imaged_planner = ImagedPipeline(cfg.imaged)

        for planner in ['modeled', 'sampled', 'simulated', 'imagined']:
            if planner == 'simulated':
                hs_sensor_info, environment_object, time_cost, path_cost, task_process, target_sequence, agent_path = variable_reset()
                set_simulation = True
                agent_state = robot.get_effector_states()
                start_point = agent_state['pos'].vec_p

                trajectory_idx = 0
                while set_simulation:
                    agent_state = robot.get_effector_states()
                    if (target_point - agent_state['pos']).length <= 0.005:
                        point_sequence += 1
                        if point_sequence == len(target_points):
                            print("complete the task!")
                            task_process = 1
                            write_data_to_csv(test_condition, planner, path_cost, time_cost, task_process)
                            break
                        else:
                            target_point = Vector(target_points[point_sequence])
                            start_point = agent_state['pos'].vec_p
                            trajectory_idx = 0
                    else:
                        pass
                    trajectory = simulation_planner.forward(hs_sensor_info, start_point[:2], target_point.vec_p)
                    print(agent_state['pos'].vec_p)
                    if trajectory.size == 0:
                        write_data_to_csv(test_condition, planner, path_cost, time_cost, task_process)
                        print('collision_false')
                        break
                    trajectory_idx += 1
                    for i in range(10):
                        safe_interact = robot.cartesian_position_controls_step(trajectory[trajectory_idx * i])

                        hs_sensor_info = tactile_collection(robot.transformMatrix(body_links), hs_sensor_info)
                        path_cost, time_cost, agent_path = evaluate_update(path_cost, time_cost, timeStep, agent_state,
                                                                           agent_path)
                        if safe_interact == False:
                            write_data_to_csv(test_condition, planner, path_cost, time_cost, task_process)
                            set_simulation = False
                            print('collision_false')
                            robot.reset_environment()
                        else:
                            pass
                        p.stepSimulation(physicsClientId=client1)
                        time.sleep(timeStep)
            else:
                pass
        p.disconnect()


                        # 判断是否要保存数据

                    # 更新传感器数据
                    # 更新规划的轨迹
                    # for 循环
                        # 更新传感器数据， 控制机器人运动，更新判断数据， 判断是不是到达局部最小值（保存数据），

        #             objects, hsSensorInfo = environment_understander.object_representation(robot.robot, robot.transformMatrix(body_links), hsSensorInfo,
        #                                                 robot.object, objects, agent_state)
        #             f_total, t, objects, f_rep, f_att, f_interact = motion_imager.solving_gradient(objects=objects, f_active=f_active, agent_state=agent_state,
        #                                                                             target_point=target_point, t=t)
        # print(f'f_total:{f_total}; f_rep:{f_rep.vec_p}; f_att:{f_att.vec_p};')
        # if objects:
        #     print(objects['object3']['attribute'], objects['object3']['states'])
        # robot.torque_control_step(f_total)
        # p.stepSimulation()
        # time.sleep(timeStep)




# if __name__ == "__main__":
#     # try:
#     #     planner_test(num_obj=0, fixed_prob=0)
#     # except KeyboardInterrupt:
#     #     pass
#     queue_traj = mp.Queue()
#     queue_result = mp.Queue()
#     queue_mission = mp.Queue()
#     queue_obstacles = mp.Queue()
#
#     num_obj = 0
#     fixed_prob = 0
#     cfg = load_config()
#
#
#     p1 = mp.Process(target=simulator_1, args=(queue_traj, queue_result, queue_mission, num_obj, fixed_prob, cfg))
#     p2 = mp.Process(target=simulator_2, args=(cfg, queue_traj, queue_result, queue_obstacles, queue_mission))
#
#     p1.start()
#     p2.start()
#
#     p1.join()
#     p2.join()


def i_mp_planner(cfg):
    object_num = random.randint(1, 5)
    coords = generate_obstacle_coords(object_num)
    fixed_prob = random.uniform(0, 1)
    robot, client1 = set_environment(coords=coords, fixed_prob=fixed_prob)



    path_lengths = [0.313, 0.605, 0.855]
    target_points = [[0.6, 0.1], [0.75, 0.35], [0.6, 0.55]]

    # load planner
    imaged_planner = ImagedPipeline(cfg.imaged)

    # set simulation
    f_active = Vector([0, 0])
    t = 0
    lines = 0
    local_time = time.time()

    environment_info, evaluate_metrics, target_sequence, agent_path = variable_reset()
    target_point = Vector(target_points[target_sequence])
    set_planning = True
    while set_planning:
        agent_state = robot.get_effector_states()
        if (target_point - agent_state['pos']).length <= 0.005:
            target_sequence += 1
            print(f'target_sequence:{target_sequence}')
            if target_sequence == len(target_points):
                print("complete the task!")
                break
            else:
                target_point = Vector(target_points[target_sequence])
        if check_time_difference(local_time) == False:
            evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost'] / path_lengths[
                len(target_points) - 1]) * 100
            print("exceed time limit")
            break
        actuation_force, t = imaged_planner.forward(robot, environment_info, agent_state, f_active,
                                                    target_point, t)
        safe_interact = robot.torque_control_step(actuation_force, agent_path)
        evaluate_metrics, agent_path = evaluate_update(evaluate_metrics, timeStep, agent_state,
                                                       agent_path)
        p.stepSimulation(physicsClientId=client1)
        time.sleep(timeStep)

    p.disconnect(physicsClientId=client1)


if __name__ == "__main__":
    cfg = load_config()
    i_mp_planner(cfg)