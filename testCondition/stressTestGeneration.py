import numpy as np
import os
import csv
import json
import random
from itertools import product



def is_inside_circle(x, y):
    center_x, center_y = 0.75, 0.336
    radius = 0.1

    dx = x - center_x
    dy = y - center_y
    distance_squared = dx ** 2 + dy ** 2

    return distance_squared <= radius ** 2

def is_point_in_quadrilateral(x, y):

    vertices = [
        (0.7, 0.45),
        (0.7, 0.281),
        (0.6, 0.342),
        (0.6, 0.39)
    ]

    # 辅助函数：判断点是否在线段上
    def is_on_segment(p, p1, p2):
        # 叉积检查共线
        cross = (p[0] - p1[0]) * (p2[1] - p1[1]) - (p[1] - p1[1]) * (p2[0] - p1[0])
        if abs(cross) > 1e-10:
            return False
        # 检查坐标是否在包围盒内
        min_x = min(p1[0], p2[0]) - 1e-10
        max_x = max(p1[0], p2[0]) + 1e-10
        min_y = min(p1[1], p2[1]) - 1e-10
        max_y = max(p1[1], p2[1]) + 1e-10
        return (min_x <= p[0] <= max_x) and (min_y <= p[1] <= max_y)

    # 检查是否在任一边上
    for i in range(4):
        p1 = vertices[i]
        p2 = vertices[(i + 1) % 4]
        if is_on_segment((x, y), p1, p2):
            return True

    # 射线法判断内部
    x_p, y_p = x, y
    inside = False
    for i in range(4):
        xi, yi = vertices[i]
        xj, yj = vertices[(i + 1) % 4]
        # 检查y是否在边的y范围内
        if (yi > y_p) != (yj > y_p):
            # 计算交点x坐标
            if yi == yj:  # 水平边已处理，跳过
                continue
            t = (y_p - yi) / (yj - yi)
            x_intersect = xi + t * (xj - xi)
            if x_intersect >= x_p:
                inside = not inside
    return inside

def generate_obstacle_coords(num_obj):
    x_min, x_max = 0.6, 1.1
    y_min, y_max = -0.2, 0.8
    num_points_x = 10
    num_points_y = 20

    x = np.linspace(x_min, x_max, num_points_x)
    y = np.linspace(y_min, y_max, num_points_y)
    X, Y = np.meshgrid(x, y)
    grid_coords = np.column_stack((X.ravel(), Y.ravel()))

    forbidden_points = np.array([[0.75, 0.366]])
    min_distance = 0.1

    distances = np.sqrt(
        (grid_coords[:, 0, np.newaxis] - forbidden_points[:, 0]) ** 2 +
        (grid_coords[:, 1, np.newaxis] - forbidden_points[:, 1]) ** 2
    )

    valid_mask = np.all(distances >= min_distance, axis=1)
    valid_coords = grid_coords[valid_mask]

    if len(valid_coords) < num_obj:
        raise ValueError(f" {num_obj} points are needed，only {len(valid_coords)} points are available.")

    selected_indices = np.random.choice(valid_coords.shape[0], size=num_obj, replace=False)
    # print(selected_indices)
    # print(valid_coords[selected_indices].tolist())
    return valid_coords[selected_indices].tolist()

test_data = []
num_obstacles_options = [6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
fixed_probability_options = [0.1, 0.2, 0.4, 0.8]

for num_obstacles, fixed_prob in product([6, 7, 8, 9, 10, 11, 12, 13, 14, 15], [0.1, 0.2, 0.4, 0.8]):
    success_count = 0
    attempt_count = 0
    max_attempts = 1000
    succeed = 0

    while success_count < 500 and attempt_count < max_attempts:
        try:
            coords = generate_obstacle_coords(num_obstacles)

            suffix = "obstacles"
            test_condition = f"{num_obstacles}_{suffix}_{int(fixed_prob * 100)}%"
            test_environments = []
            for coord in coords:
                fixed_base = 1 if random.random() < fixed_prob else 0
                coord.append(fixed_base)
                test_environments.append(coord)
            completed = 1
            for test_environment in test_environments:
                if test_environment[2] == 1:
                    if is_point_in_quadrilateral(test_environment[0], test_environment[1]) or is_inside_circle(test_environment[0], test_environment[1]):
                        completed = 0
                else:
                    pass
            if completed == 1:
                succeed += 1
            test_data.append({
                "test_condition": test_condition,
                "num_obstacles": num_obstacles,
                "fixed_prob" : fixed_prob,
                "test_environments": json.dumps(test_environments),
                "mission to be completed": completed,
            })
            success_count += 1
            if success_count == 499:
                succeed_rate = succeed/(success_count+1)
                print(f"num_obstacles{num_obstacles}_fixed_prob{fixed_prob}:{succeed_rate}")
        except ValueError as e:
            print(f"重试中: {str(e)}")
            attempt_count += 1
            continue

file_path = '../testCondition/stress_test_conditions.csv'
os.makedirs(os.path.dirname(file_path), exist_ok=True)

with open(file_path, 'w', newline='') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=['test_condition', 'num_obstacles', "fixed_prob",
                                                 'test_environments', "mission to be completed"])
    writer.writeheader()
    writer.writerows(test_data)

print("测试条件已生成至 .testCondition/stress_test_conditions.csv")