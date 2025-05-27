import csv
import time
import math
import json
import argparse
import numpy as np
import pybullet as p
import pybullet_data as pd
import multiprocessing as mp
from omegaconf import OmegaConf
from os.path import join, dirname, abspath

from motionPlanner.vector import Vector
from utils.utils import write_data_to_csv, evaluate_update, check_time_difference
from motionPlanner.motionPlanning import MotionImagination
from robotEnvironment.robotEnvironmentConfig import Robot
from environmentUnderstanding.approachingPerception import tactile_collection
from environmentUnderstanding.dataProcess import EnvironmentUnderstanding
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

def set_environment():
    client1 = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.setPhysicsEngineParameter(numSolverIterations=50)
    p.setGravity(0, 0, -9.8, physicsClientId=client1)
    p.setTimeStep(timeStep)
    robot_urdf = join(dirname(abspath(__file__)), 'envDescription/single_arm/left_arm.urdf')
    robot = Robot(bullet_client=p, path=robot_urdf, clint_id=client1)
    return robot, client1

def variable_reset(task_difficulty):
    environment_info = {"point_cloud_sets":[], "hs_sensor_info":{}, "objects":{}}
    evaluate_metrics = {"time_cost":0.0, "path_cost":0, "task_progress":0, "task_difficulty": task_difficulty}
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

def mission_config(robot, target_points):
    agent_state = robot.get_effector_states()
    start_point = agent_state['pos'].vec_p[:2]
    target_point = Vector(target_points[0])
    return start_point, target_point

def simulator_1(queue_traj, queue_result, queue_mission, queue_obstacles, cfg):

    robot, client1 = set_environment()

    path_lengths = [0.313, 0.605, 0.855]

    # load planner
    simulation_planner = SimulatedPipeline(cfg.simulated)
    lines = 0

    with open('./testCondition/test_conditions.csv') as f:
        reader = csv.DictReader(f)
        for row in reader:
            lines += 1
            if lines > 0 and lines <= 10800:
                coordinates = json.loads(row['coordinates'])
                fix_prob = float(row['fixed_probability'])
                target_points = json.loads(row['target_points'])
                task_difficulty = float(row['task_difficulty'])
                print("\n\n\ntask_num:", lines)
                local_time = time.time()
                formatted_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(local_time))
                print("current time:", formatted_time)
                robot.init_environment()
                robot.load_environment(coordinates, fix_prob)
                environment_info, evaluate_metrics, target_sequence, agent_path = variable_reset(task_difficulty)
                start_point, target_point = mission_config(robot, target_points)
                trajectory_idx = 0
                set_planning = True
                while set_planning:
                    agent_state = robot.get_effector_states()
                    if (target_point - agent_state['pos']).length <= 0.015:
                        target_sequence += 1
                        if target_sequence == len(target_points):
                            print("complete the task!")
                            evaluate_metrics['task_progress'] = 1
                            evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost']/path_lengths[len(target_points)-1])*100
                            write_data_to_csv(row['test_condition'], 'simulation_based', evaluate_metrics)
                            queue_mission.put(True)
                            break
                        else:
                            target_point = Vector(target_points[target_sequence])
                            start_point = agent_state['pos'].vec_p
                            trajectory_idx = 0
                    else:
                        pass
                    if check_time_difference(local_time) == False:
                        evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost'] / path_lengths[
                            len(target_points) - 1]) * 100
                        write_data_to_csv(row['test_condition'], 'model_based', evaluate_metrics)
                        break
                    trajectory = simulation_planner.forward(environment_info, start_point[:2],
                                                            target_point.vec_p, queue_traj,
                                                            queue_result, queue_obstacles)
                    if trajectory.size == 0:
                        write_data_to_csv(row['test_condition'], 'simulation_based', evaluate_metrics)
                        queue_mission.put(True)
                        print('collision_false')
                        break
                    for i in range(trajectory_idx, trajectory_idx + 50):
                        safe_interact = robot.cartesian_position_controls_step(trajectory[i])
                        environment_info = tactile_collection(robot.transformMatrix(cfg.simulated.body_link),
                                                              environment_info)
                        evaluate_metrics, agent_path = evaluate_update(evaluate_metrics, timeStep, agent_state,
                                                                       agent_path)
                        if safe_interact == False:
                            print(safe_interact)
                            write_data_to_csv(row['test_condition'], 'simulation_based', evaluate_metrics)
                            queue_mission.put(True)
                            print('collision_false')
                            set_planning = False
                            break
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
            else:
                print('\nBaseline comparison have been completed.')
                pass
            p.disconnect(physicsClientId=client1)

def i_mp_planner(cfg):
    robot, client1 = set_environment()

    path_lengths = [0.313, 0.605, 0.855]

    # load planner
    imaged_planner = ImagedPipeline(cfg.imaged)

    # set simulation
    f_active = Vector([0, 0])
    t = 0
    lines = 0

    with open('./testCondition/test_conditions.csv') as f:
        reader = csv.DictReader(f)
        for row in reader:
            lines += 1
            if lines > 0 and lines <= 10800:
                coordinates = json.loads(row['coordinates'])
                fix_prob = float(row['fixed_probability'])
                target_points = json.loads(row['target_points'])
                task_difficulty = float(row['task_difficulty'])
                print("\n\ntask_num:", lines)
                local_time = time.time()
                formatted_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(local_time))
                print("current time: ", formatted_time)
                robot.init_environment()
                robot.load_environment(coordinates, fix_prob)
                environment_info, evaluate_metrics, target_sequence, agent_path = variable_reset(task_difficulty)
                target_point = Vector(target_points[target_sequence])
                set_planning = True
                while set_planning:
                    agent_state = robot.get_effector_states()
                    if (target_point - agent_state['pos']).length <= 0.005:
                        target_sequence += 1
                        print(f'target_sequence:{target_sequence}')
                        if target_sequence == len(target_points):
                            print("complete the task!")
                            evaluate_metrics['task_progress'] = 1
                            evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost'] / path_lengths[len(target_points) - 1]) * 100
                            write_data_to_csv(row['test_condition'], 'I-MP_based', evaluate_metrics)
                            break
                        else:
                            target_point = Vector(target_points[target_sequence])
                    if check_time_difference(local_time) == False:
                        evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost'] / path_lengths[
                            len(target_points) - 1]) * 100
                        write_data_to_csv(row['test_condition'], 'model_based', evaluate_metrics)
                        print("exceed time limit")
                        break
                    actuation_force, t = imaged_planner.forward(robot, environment_info, agent_state, f_active,
                                                             target_point, t)
                    safe_interact = robot.torque_control_step(actuation_force, agent_path)
                    evaluate_metrics, agent_path = evaluate_update(evaluate_metrics, timeStep, agent_state,
                                                                   agent_path)
                    if safe_interact == False:
                        evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost'] / path_lengths[
                            len(target_points) - 1]) * 100
                        write_data_to_csv(row['test_condition'], 'I-MP_based', evaluate_metrics)
                        print("contact force false")
                        break
                    else:
                        pass
                    p.stepSimulation(physicsClientId=client1)
                    time.sleep(timeStep)
            else:
                print('\nBaseline comparison have been completed.')
                pass
        p.disconnect(physicsClientId=client1)

def probability_planner(cfg):
    robot, client1 = set_environment()

    path_lengths = [0.313, 0.605, 0.855]
    lines = 0

    # load planner
    sampled_planner = SampledPipeline(cfg.sampled)

    with open('./testCondition/test_conditions.csv') as f:
        reader = csv.DictReader(f)
        for row in reader:
            lines += 1
            if lines > 0 and lines <= 10800:
                coordinates = json.loads(row['coordinates'])
                fix_prob = float(row['fixed_probability'])
                target_points = json.loads(row['target_points'])
                task_difficulty = float(row['task_difficulty'])
                print("\n\ntask_num:", lines)
                local_time = time.time()
                formatted_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(local_time))
                print("current time: ", formatted_time)
                robot.init_environment()
                robot.load_environment(coordinates, fix_prob)
                environment_info, evaluate_metrics, target_sequence, agent_path = variable_reset(task_difficulty)
                target_point = Vector(target_points[target_sequence])
                set_planning = True
                while set_planning:
                    safe_interact, contact_force = robot._detect_collision_force()
                    agent_state = robot.get_effector_states()
                    if (target_point - agent_state['pos']).length <= 0.005:
                        target_sequence += 1
                        if target_sequence == len(target_points):
                            print("the last target point!")
                            evaluate_metrics['task_progress'] = 1
                            evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost'] / path_lengths[
                                len(target_points) - 1]) * 100
                            write_data_to_csv(row['test_condition'], 'probability_based', evaluate_metrics)
                            break
                        else:
                            target_point = Vector(target_points[target_sequence])
                    else:
                        pass
                    if check_time_difference(local_time) == False:
                        evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost'] / path_lengths[
                            len(target_points) - 1]) * 100
                        write_data_to_csv(row['test_condition'], 'model_based', evaluate_metrics)
                        break
                    trajectory = sampled_planner.forward(environment_info, agent_state['pos'].vec_p, target_point.vec_p)
                    if not trajectory:
                        evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost'] / path_lengths[
                            len(target_points) - 1]) * 100
                        write_data_to_csv(row['test_condition'], 'probability_based', evaluate_metrics)
                        break
                    if len(trajectory) >= 10:
                        robot.set_impedance_parameter(kp=[10, 10], kd=[3, 3])
                        set_control = True
                        i = 0
                        while set_control:
                            agent_state = robot.get_effector_states()
                            if (Vector(list(trajectory[i])) - agent_state['pos']).length <= 0.003:
                                i += 1
                            if i > 5:
                                break
                            safe_interact, contact_force = robot.impedance_control_step(trajectory[i],
                                                                                        cfg.sampled.desire_velocity,
                                                                                        agent_state['pos'].vec_p,
                                                                                        agent_state['vel'].vec_p,
                                                                                        contact_force,
                                                                                        agent_path)
                            environment_info = tactile_collection(robot.transformMatrix(cfg.sampled.body_link), environment_info)
                            evaluate_metrics, agent_path = evaluate_update(evaluate_metrics, timeStep, agent_state,
                                                                           agent_path)
                            if safe_interact == False:
                                evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost'] / path_lengths[
                                    len(target_points) - 1]) * 100
                                write_data_to_csv(row['test_condition'], 'probability_based', evaluate_metrics)
                                set_planning = False
                                break
                            else:
                                pass
                            if check_time_difference(local_time) == False:
                                evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost'] / path_lengths[
                                    len(target_points) - 1]) * 100
                                write_data_to_csv(row['test_condition'], 'model_based', evaluate_metrics)
                                break
                            p.stepSimulation()
                            time.sleep(timeStep)
                    else:
                        robot.set_impedance_parameter(kp=[10, 10], kd=[3, 3])
                        for position in trajectory:
                            set_control = True
                            while set_control:
                                agent_state = robot.get_effector_states()
                                if (Vector(list(position)) - agent_state['pos']).length <= 0.003:
                                    break
                                else:
                                    safe_interact, contact_force = robot.impedance_control_step(position,
                                                                                                cfg.sampled.desire_velocity,
                                                                                                agent_state['pos'].vec_p,
                                                                                                agent_state['vel'].vec_p,
                                                                                                contact_force, agent_path)
                                    environment_info = tactile_collection(robot.transformMatrix(cfg.sampled.body_link),
                                                                          environment_info)
                                    evaluate_metrics, agent_path = evaluate_update(evaluate_metrics, timeStep, agent_state,
                                                                                   agent_path)
                                    if safe_interact == False:
                                        evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost'] / path_lengths[
                                            len(target_points) - 1]) * 100
                                        write_data_to_csv(row['test_condition'], 'probability_based', evaluate_metrics)
                                        set_planning = False
                                        break
                                    else:
                                        pass
                                    if check_time_difference(local_time) == False:
                                        evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost'] / path_lengths[
                                            len(target_points) - 1]) * 100
                                        write_data_to_csv(row['test_condition'], 'model_based', evaluate_metrics)
                                        break
                                    p.stepSimulation()
                                    time.sleep(timeStep)
            else:
                print('\nBaseline comparison have been completed.')
                pass
            p.disconnect(physicsClientId=client1)

def model_planner(cfg):
    robot, client1 = set_environment()

    path_lengths = [0.313, 0.605, 0.855]
    lines = 0

    # load planner
    modeled_planner = ModeledPipeline(cfg.modeled)

    with open('./testCondition/test_conditions.csv') as f:
        reader = csv.DictReader(f)
        for row in reader:
            lines += 1
            if lines > 0 and lines <= 10800:
                coordinates = json.loads(row['coordinates'])
                fix_prob = float(row['fixed_probability'])
                target_points = json.loads(row['target_points'])
                task_difficulty = float(row['task_difficulty'])
                print("\n\ntask_num:", lines)
                local_time = time.time()
                formatted_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(local_time))
                print("current time：", formatted_time)
                robot.init_environment()
                robot.load_environment(coordinates, fix_prob)
                environment_info, evaluate_metrics, target_sequence, agent_path = variable_reset(task_difficulty)
                target_point = Vector(target_points[target_sequence])
                set_planning = True
                while set_planning:
                    agent_state = robot.get_effector_states()
                    if (target_point - agent_state['pos']).length <= 0.005:
                        target_sequence += 1
                        if target_sequence == len(target_points):
                            print("the last target point!")
                            evaluate_metrics['task_progress'] = 1
                            evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost'] / path_lengths[
                                len(target_points) - 1]) * 100
                            write_data_to_csv(row['test_condition'], 'model_based', evaluate_metrics)
                            break
                        else:
                            target_point = Vector(target_points[target_sequence])
                    else:
                        pass
                    if check_time_difference(local_time) == False:
                        evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost'] / path_lengths[
                            len(target_points) - 1]) * 100
                        write_data_to_csv(row['test_condition'], 'model_based', evaluate_metrics)
                        break
                    trajectory = modeled_planner.forward(environment_info, agent_state['pos'].vec_p, target_point.vec_p)
                    if not trajectory:
                        evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost'] / path_lengths[
                            len(target_points) - 1]) * 100
                        write_data_to_csv(row['test_condition'], 'model_based', evaluate_metrics)
                        break
                    for position in trajectory:
                        safe_interact = robot.cartesian_position_controls_step(position)
                        environment_info = tactile_collection(robot.transformMatrix(cfg.sampled.body_link),
                                                              environment_info)
                        evaluate_metrics, agent_path = evaluate_update(evaluate_metrics, timeStep, agent_state,
                                                                       agent_path)
                        if safe_interact == False:
                            evaluate_metrics['path_cost'] = (evaluate_metrics['path_cost'] / path_lengths[
                                len(target_points) - 1]) * 100
                            write_data_to_csv(row['test_condition'], 'model_based', evaluate_metrics)
                            set_planning = False
                            break
                        else:
                            pass
                        p.stepSimulation()
                        time.sleep(timeStep)
            else:
                print('\nBaseline comparison have been completed.')
                pass
            p.disconnect(physicsClientId=client1)

def simulation_planner(cfg):

    queue_traj = mp.Queue()
    queue_result = mp.Queue()
    queue_mission = mp.Queue()
    queue_obstacles = mp.Queue()

    p1 = mp.Process(target=simulator_1, args=(queue_traj, queue_result, queue_mission, queue_obstacles, cfg))
    p2 = mp.Process(target=simulator_2, args=(cfg, queue_traj, queue_result, queue_obstacles, queue_mission))

    p1.start()
    p2.start()

    p1.join()
    p2.join()


if __name__ == '__main__':
    cfg = load_config()
    i_mp_planner(cfg)
    probability_planner(cfg)
    model_planner(cfg)
    simulation_planner(cfg)