import os
import csv
import time
import numpy as np
from datetime import datetime
from os.path import join, dirname, abspath

def write_data_to_csv(test_condition, planner, evaluate_metrics):
    file_path = join(dirname(dirname(abspath(__file__))), f'data/baseline_comparison_{planner}.csv')
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, 'a', newline='') as file:
        writer = csv.writer(file)
        if file.tell() == 0:
            writer.writerow(["test_condition", "planner", "task_difficulty", "path_cost(m)", "time_cost(s)", 'task_progress'])
        writer.writerow([str(test_condition), planner, str(evaluate_metrics['task_difficulty']), str(evaluate_metrics['path_cost']), str(evaluate_metrics['time_cost']), str(evaluate_metrics['task_progress'])])

def write_thread_data_to_csv(test_condition, planner, evaluate_metrics, thread):
    file_path = join(dirname(dirname(abspath(__file__))), f'data/baseline_comparison_{planner}_{thread}.csv')
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, 'a', newline='') as file:
        writer = csv.writer(file)
        if file.tell() == 0:
            writer.writerow(["test_condition", "planner", "task_difficulty", "path_cost(m)", "time_cost(s)", 'task_progress'])
        writer.writerow([str(test_condition), planner, str(evaluate_metrics['task_difficulty']), str(evaluate_metrics['path_cost']), str(evaluate_metrics['time_cost']), str(evaluate_metrics['task_progress'])])

def path_eval(agent_path, path_cost):
    path_cost += np.sum(np.abs(np.array(agent_path[-1]) - np.array(agent_path[-2])))
    return path_cost

def time_eval(time_cost, time_step):
    time_cost += time_step
    return time_cost

def evaluate_update(evaluate_metrics, time_step, agent_state, agent_path):
    agent_path.append(agent_state['pos'].vec_p)
    if len(agent_path) == 1:
        pass
    else:
        evaluate_metrics['path_cost'] = path_eval(agent_path, evaluate_metrics['path_cost'])
    evaluate_metrics['time_cost'] = time_eval(evaluate_metrics['time_cost'], time_step)
    return evaluate_metrics, agent_path

def check_time_difference(start_time):

    current_time = time.time()
    if isinstance(start_time, datetime):
        start_time = start_time.timestamp()
    time_diff = current_time - start_time

    return time_diff <= 180


def write_stress_test_to_csv(test_condition, object_num, fixed_prob, task_states, mission_to_be_completed, thread):
    file_path = join(dirname(dirname(abspath(__file__))), f'data/stress_testing_thread{thread}.csv')
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, 'a', newline='') as file:
        writer = csv.writer(file)
        if file.tell() == 0:
            writer.writerow(
                ["test_condition", "num_object", "fixed_prob", "stak_progress", "mission_to_be_completed"])
        writer.writerow(
            [str(test_condition), str(object_num), str(fixed_prob), str(task_states), str(mission_to_be_completed)])

def write_knocking_over_to_csv(test_condition, object_num, fixed_prob, task_states, mission_to_be_completed, thread):
    file_path = join(dirname(dirname(abspath(__file__))), f'data/knocking_over_thread{thread}.csv')
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, 'a', newline='') as file:
        writer = csv.writer(file)
        if file.tell() == 0:
            writer.writerow(
                ["test_condition", "num_object", "fixed_prob", "stak_progress", "mission_to_be_completed"])
        writer.writerow(
            [str(test_condition), str(object_num), str(fixed_prob), str(task_states), str(mission_to_be_completed)])

def write_sensory_failure_data_to_csv(failure_mode, time, effector_vel, contact_force):
    file_path = join(dirname(dirname(abspath(__file__))), f'data/sensory_failure_experiments_{failure_mode}.csv')
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, 'a', newline='') as file:
        writer = csv.writer(file)
        if file.tell() == 0:
            writer.writerow(
                ["time", "effector_velocity", "contact_force"])
        writer.writerow(
            [str(time), str(effector_vel), str(contact_force)])

