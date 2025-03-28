import time
import argparse
import numpy as np
import pybullet as p
import pybullet_data as pd
from omegaconf import OmegaConf
from os.path import join, dirname, abspath
from motionPlanner.vector import Vector
from utils.utils import write_data_to_csv, evaluate_update
from motionPlanner.motionPlanning import MotionImagination
from robotEnvironment.robotEnvironmentConfig import Robot
from environmentUnderstanding.approachingPerception import tactile_collection
from environmentUnderstanding.dataProcess import EnvironmentUnderstanding
from compared_method.plannerPipeline import ModeledPipeline, SimulatedPipeline, SampledPipeline

timeStep = 1 / 120

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


def set_environment(obj_num, fixed_prob):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(timeStep)
    robot_urdf = join(dirname(abspath(__file__)), 'envDescription/single_arm/left_arm.urdf')
    robot = Robot(bullet_client=p, path=robot_urdf, obj_num=obj_num, fixed_prob=fixed_prob)
    return robot

def variable_reset():
    hs_sensor_info = []
    environment_object = {}
    time_cost = 0.0
    task_process = 0
    path_cost = 0
    target_sequence = 0
    agent_path = []
    return hs_sensor_info, environment_object, time_cost, path_cost, task_process, target_sequence, agent_path

def planner_test(num_obj, fixed_prob):
    test_condition = []
    test_condition.append(f'object number: {str(num_obj)}')
    test_condition.append(f'fixation probability: {str(fixed_prob)}')

    # motion task
    target_points = [[0.55, 0.1], [0.75, 0.35], [0.55, 0.55]]
    test_condition.append(f'task difficulty: {len(target_points)}')
    point_sequence = 0
    target_point = Vector(target_points[point_sequence])

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
        robot = set_environment(num_obj, fixed_prob)
        # robot.load_environment(num_obj=num_obj, fixed_prob=fixed_prob)

        # motion task
        target_points = [[0.58, 0.15], [0.75, 0.35], [0.55, 0.55]]

        # load_planner
        model_planner = ModeledPipeline(cfg.modeled)
        # simulation_planner = SimulatedPipeline(cfg.simulated)
        sampled_planner = SampledPipeline(cfg.sampled)

        for planner in ['modeled', 'sampled', 'simulated', 'imagined']:
            if planner == 'modeled':
                hs_sensor_info, environment_object, time_cost, path_cost, task_process, target_sequence, agent_path = variable_reset()
                set_simulation = True
                while set_simulation:
                    agent_state = robot.get_effector_states()
                    if (target_point - agent_state['pos']).length <= 0.005:
                        point_sequence += 1
                        if point_sequence == len(target_points):
                            print("the last target point!")
                            task_process = 1
                            write_data_to_csv(test_condition, planner, path_cost, time_cost, task_process)
                            break
                        else:
                            target_point = Vector(target_points[point_sequence])
                    else:
                        pass
                    trajectory = model_planner.forward(hs_sensor_info, agent_state['pos'].vec_p, target_point.vec_p)
                    for position in trajectory:
                        safe_interact = robot.cartesian_position_controls_step(position)
                        print(f'act:{safe_interact}')
                        hs_sensor_info = tactile_collection(robot.transformMatrix(body_links), hs_sensor_info)
                        path_cost, time_cost, agent_path = evaluate_update(path_cost, time_cost, timeStep, agent_state, agent_path)
                        if safe_interact == False:
                            write_data_to_csv(test_condition, planner, path_cost, time_cost, task_process)
                            set_simulation = False
                            robot.reset_environment()
                            break
                        else:
                            pass
                        p.stepSimulation()
                        time.sleep(timeStep)

            elif planner == 'sampled':
                hs_sensor_info, environment_object, time_cost, path_cost, task_process, target_sequence, agent_path = variable_reset()
                set_simulation = True
                while set_simulation:
                        agent_state = robot.get_effector_states()
                        safe_interact, contact_force = robot._detect_collision_force()
                        if (target_point - agent_state['pos']).length <= 0.005:
                            point_sequence += 1
                            if point_sequence == len(target_points):
                                print("the last target point!")
                                task_process = 1
                                write_data_to_csv(test_condition, planner, path_cost, time_cost, task_process)
                                break
                            else:
                                target_point = Vector(target_points[point_sequence])
                        else:
                            pass
                        trajectory = sampled_planner.forward(hs_sensor_info, agent_state['pos'].vec_p,
                                                             target_point.vec_p)
                        if not trajectory:
                            write_data_to_csv(test_condition, planner, path_cost, time_cost, task_process)
                            break
                        if len(target_points) >= 10:
                            for i in range(5):
                                safe_interact, contact_force = robot.impedance_control_step(trajectory[i],
                                                                                            cfg.sampled.desire_velocity,
                                                                                            agent_state['pos'].vec_p,
                                                                                            agent_state['vel'].vec_p,
                                                                                            contact_force)

                                print(f'act:{safe_interact}')
                                hs_sensor_info = tactile_collection(robot.transformMatrix(body_links), hs_sensor_info)
                                path_cost, time_cost, agent_path = evaluate_update(path_cost, time_cost, timeStep,
                                                                                   agent_state,
                                                                                   agent_path)
                                if safe_interact == False:
                                    write_data_to_csv(test_condition, planner, path_cost, time_cost, task_process)
                                    set_simulation = False
                                    robot.reset_environment()
                                    break
                                else:
                                    pass
                                p.stepSimulation()
                                time.sleep(timeStep)
                        else:
                            for position in trajectory:
                                safe_interact, contact_force = robot.impedance_control_step(position,
                                                                                            cfg.sampled.desire_velocity,
                                                                                            agent_state['pos'].vec_p,
                                                                                            agent_state['vel'].vec_p,
                                                                                            contact_force)

                                print(f'act:{safe_interact}')
                                hs_sensor_info = tactile_collection(robot.transformMatrix(body_links), hs_sensor_info)
                                path_cost, time_cost, agent_path = evaluate_update(path_cost, time_cost, timeStep,
                                                                                   agent_state,
                                                                                   agent_path)
                                if safe_interact == False:
                                    write_data_to_csv(test_condition, planner, path_cost, time_cost, task_process)
                                    set_simulation = False
                                    robot.reset_environment()
                                    break
                                else:
                                    pass
                                p.stepSimulation()
                                time.sleep(timeStep)

            elif planner == 'simulated':
        p.disconnect()

        for i in range(20):

            # load environment
            robot, client1 = set_environment(num_obj, fixed_prob)
            # robot.load_environment(num_obj=num_obj, fixed_prob=fixed_prob)

            # motion task
            # target_points = [[0.58, 0.15], [0.75, 0.35], [0.55, 0.55]]

            # load_planner
            model_planner = ModeledPipeline(cfg.modeled)

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
                            environment_info = tactile_collection(robot.transformMatrix(cfg.simulated.body_link),
                                                                  environment_info)
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

   


if __name__ == "__main__":
    try:
        planner_test(num_obj=0, fixed_prob=0)
    except KeyboardInterrupt:
        pass