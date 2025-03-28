import os
import csv
import numpy as np
from os.path import join, dirname, abspath

def write_data_to_csv(test_condition, planner, task_difficulty, evaluate_metrics):
    file_path = join(dirname(dirname(abspath(__file__))), f'data/statistical_performance_{planner}_{task_difficulty}.csv')
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, 'a', newline='') as file:
        writer = csv.writer(file)
        if file.tell() == 0:
            writer.writerow(["test_condition", "planner", "task_difficulty", "path_cost(m)", "time_cost(s)", 'task_progress'])
        writer.writerow([str(test_condition), planner, str(task_difficulty), str(evaluate_metrics['path_cost']), str(evaluate_metrics['time_cost']), str(evaluate_metrics['task_progress'])])

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

