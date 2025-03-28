import numpy as np
import os
import csv
import json
from itertools import product


def generate_obstacle_coords(num_obj):
    x_min, x_max = 0.5, 0.8
    y_min, y_max = 0.1, 0.5
    num_points_x = 4
    num_points_y = 5

    x = np.linspace(x_min, x_max, num_points_x)
    y = np.linspace(y_min, y_max, num_points_y)
    X, Y = np.meshgrid(x, y)
    grid_coords = np.column_stack((X.ravel(), Y.ravel()))

    forbidden_points = np.array([[0.6, 0.1], [0.75, 0.35], [0.6, 0.55]])
    min_distance = 0.05

    distances = np.sqrt(
        (grid_coords[:, 0, np.newaxis] - forbidden_points[:, 0]) ** 2 +
        (grid_coords[:, 1, np.newaxis] - forbidden_points[:, 1]) ** 2
    )


    valid_mask = np.all(distances >= min_distance, axis=1)
    valid_coords = grid_coords[valid_mask]


    if len(valid_coords) < num_obj:
        raise ValueError(f" {num_obj} points are needed，only {len(valid_coords)} points are available.")


    selected_indices = np.random.choice(valid_coords.shape[0], size=num_obj, replace=False)
    return valid_coords[selected_indices].tolist()


test_data = []
num_obstacles_options = [1, 3, 6]
fixed_probability_options = [0, 0.5, 1.0]

for num_obstacles, fixed_prob in product(num_obstacles_options, fixed_probability_options):
    success_count = 0
    attempt_count = 0
    max_attempts = 30

    while success_count < 10 and attempt_count < max_attempts:
        try:
            coords = generate_obstacle_coords(num_obstacles)

            suffix = "obstacle" if num_obstacles == 1 else "obstacles"
            test_condition = f"{num_obstacles}_{suffix}_{int(fixed_prob * 100)}%"

            test_data.append({
                "test_condition": test_condition,
                "num_obstacles": num_obstacles,
                "fixed_probability": fixed_prob,
                "coordinates": json.dumps(coords),
                "fixed_prob_value": fixed_prob
            })
            success_count += 1
        except ValueError as e:
            print(f"重试中: {str(e)}")
            attempt_count += 1
            continue

file_path = '../testCondition/test_conditions.csv'
os.makedirs(os.path.dirname(file_path), exist_ok=True)

with open(file_path, 'w', newline='') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=['test_condition', 'num_obstacles',
                                                 'fixed_probability', 'coordinates', 'fixed_prob_value'])
    writer.writeheader()
    writer.writerows(test_data)

print("测试条件已生成至 .testCondition/test_conditions.csv")