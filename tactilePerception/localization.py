
import pybullet as p
import numpy as np
import math


def localization(bodyId, avaliable_Link_index):

    orn_list1 = []
    orn_list2 = []
    obj_point = np.empty((1, 2), dtype=float)

    joint_trans = np.eye(4)
    trans_matrix = np.eye(4)
    trans_group1 = []
    trans_group2 = []
    trans_group3 = []
    trans_group4 = []
    trans_link = []
    start_point = np.empty((1, 3), dtype=float)
    end_point = np.empty((1, 3), dtype=float)
    touch = []
    print("hallow")
    for i in [0.0455, 0.0]:  # wrist1
        if i == 0.0455:
            sub_trans_matrix = []
            for j in [1.701748, 2.6626, 3.62156, 4.58149]:
                first_line = [0.0, 0.0, i]
                rot_matrix1 = p.getMatrixFromQuaternion([0.0, 0.0, math.sin(j / 2), math.cos(j / 2)])
                trans_matrix[:3, :3] = np.array(rot_matrix1).reshape(3, 3)
                trans_matrix[:3, 3] = np.array(first_line)
                sub_trans_matrix.append(trans_matrix.copy())
            trans_group1.append(sub_trans_matrix)
        else:
            sub_trans_matrix = []
            for j in [1.22179, 2.18169, 3.14159, 4.10149, 5.06139]:
                first_line = [0.0, 0.0, i]
                rot_matrix1 = p.getMatrixFromQuaternion([0.0, 0.0, math.sin(j / 2), math.cos(j / 2)])
                trans_matrix[:3, :3] = np.array(rot_matrix1).reshape(3, 3)
                trans_matrix[:3, 3] = np.array(first_line)
                sub_trans_matrix.append(trans_matrix.copy())
            trans_group1.append(sub_trans_matrix)

    for i in [0.0185, -0.0265]:  # wrist2
        if i == 0.0185:
            sub_trans_matrix = []
            for j in [1.701748, 2.6626, 3.62156, 4.58149]:
                first_line = [0.0, 0.0, i]
                rot_matrix1 = p.getMatrixFromQuaternion(
                    [0.0, 0.0, math.sin((j - 1.5708) / 2), math.cos((j - 1.5708) / 2)])
                trans_matrix[:3, :3] = np.array(rot_matrix1).reshape(3, 3)
                trans_matrix[:3, 3] = np.array(first_line)
                sub_trans_matrix.append(trans_matrix.copy())
            trans_group2.append(sub_trans_matrix)

        else:
            sub_trans_matrix = []
            for j in [1.22179, 2.18169, 3.14159, 4.10149, 5.06139]:
                first_line = [0.0, 0.0, i]
                rot_matrix1 = p.getMatrixFromQuaternion(
                    [0.0, 0.0, math.sin((j - 1.5708) / 2), math.cos((j - 1.5708) / 2)])
                trans_matrix[:3, :3] = np.array(rot_matrix1).reshape(3, 3)
                trans_matrix[:3, 3] = np.array(first_line)
                sub_trans_matrix.append(trans_matrix.copy())
            trans_group2.append(sub_trans_matrix)

    for i in [0.0185, -0.0265]:  # wrist3[1. 0. 0. 0.]
        if i == 0.0185:
            sub_trans_matrix = []
            for j in [1.701748, 2.6626, 3.62156, 4.58149]:
                first_line = [0.0, 0.0, i]
                rot_matrix1 = p.getMatrixFromQuaternion(
                    [0.0, 0.0, math.sin((j + 1.5708) / 2), math.cos((j + 1.5708) / 2)])
                trans_matrix[:3, :3] = np.array(rot_matrix1).reshape(3, 3)
                trans_matrix[:3, 3] = np.array(first_line)
                sub_trans_matrix.append(trans_matrix.copy())
            trans_group3.append(sub_trans_matrix)
        else:
            sub_trans_matrix = []
            for j in [1.22179, 2.18169, 3.14159, 4.10149, 5.06139]:
                first_line = [0.0, 0.0, i]
                rot_matrix1 = p.getMatrixFromQuaternion(
                    [0.0, 0.0, math.sin((j + 1.5708) / 2), math.cos((j + 1.5708) / 2)])
                trans_matrix[:3, :3] = np.array(rot_matrix1).reshape(3, 3)
                trans_matrix[:3, 3] = np.array(first_line)
                sub_trans_matrix.append(trans_matrix.copy())
            trans_group3.append(sub_trans_matrix)

    for i in [-0.0185, -0.057]:  # wrist4
        if i == -0.0185:
            sub_trans_matrix = []
            for j in [0.0, 1.04719, 2.0944, 3.14159, 4.18879, 5.235987]:
                first_line = [0.0, 0.0, i]
                j += 0.5235
                rot_matrix1 = p.getMatrixFromQuaternion([0.0, 0.0, math.sin(j / 2), math.cos(j / 2)])
                trans_matrix[:3, :3] = np.array(rot_matrix1).reshape(3, 3)
                trans_matrix[:3, 3] = np.array(first_line)
                sub_trans_matrix.append(trans_matrix.copy())
            trans_group4.append(sub_trans_matrix)
        else:
            first_line = [0.0, 0.0, i]
            trans_matrix[:3, :3] = np.eye(3)
            trans_matrix[:3, 3] = np.array(first_line)
            trans_group4.append(trans_matrix.copy())

# get trans matrix about reach joint pose
    for i in avaliable_Link_index[1:]:
        print("the call joint:".format(i))
        joint_state = p.getLinkState(bodyUniqueId=bodyId, linkIndex=i)
        joint_pos, joint_ori = joint_state[4], joint_state[5]
        rot_matrix = p.getMatrixFromQuaternion(joint_ori)
        joint_trans[:3, :3] = np.array(rot_matrix).reshape(3, 3)
        joint_trans[:3, 3] = np.array(joint_pos)
        trans_link.append(joint_trans.copy())


    for i in range(len(trans_link)):
        if i == 0:
            for j in [0, 1]:
                if j == 0:
                    point_s = []
                    point_e = []
                    for k in range(4):
                        trans = np.dot(trans_link[i], trans_group1[j][k])
                        point1 = np.dot(trans, [0, 0, 0, 1])
                        point2 = np.dot(trans, [0.2, 0, 0, 1])
                        point_s.append(point1[:3].copy())
                        point_e.append(point2[:3].copy())
                    start_point = np.append(start_point, point_s.copy())
                    end_point = np.append(end_point, point_e.copy())
            else:
                point_s = []
                point_e = []
                for k in range(5):
                    trans = np.dot(trans_link[i], trans_group1[j][k])
                    point1 = np.dot(trans, [0, 0, 0, 1])
                    point2 = np.dot(trans, [0.2, 0, 0, 1])
                    point_s.append(point1[:3].copy())
                    point_e.append(point2[:3].copy())
                start_point = np.append(start_point, point_s.copy())
                end_point = np.append(end_point, point_e.copy())

        elif i == 1:
            for j in [0, 1]:
                if j == 0:
                    point_s = []
                    point_e = []
                    for k in range(4):
                        trans = np.dot(trans_link[i], trans_group2[j][k])
                        point1 = np.dot(trans, [0, 0, 0, 1])
                        point2 = np.dot(trans, [0.2, 0, 0, 1])
                        point_s.append(point1[:3].copy())
                        point_e.append(point2[:3].copy())
                    start_point = np.append(start_point, point_s.copy())
                    end_point = np.append(end_point, point_e.copy())
                else:
                    point_s = []
                    point_e = []
                    for k in range(5):
                        trans = np.dot(trans_link[i], trans_group2[j][k])
                        point1 = np.dot(trans, [0, 0, 0, 1])
                        point2 = np.dot(trans, [0.2, 0, 0, 1])
                        point_s.append(point1[:3].copy())
                        point_e.append(point2[:3].copy())
                    start_point = np.append(start_point, point_s.copy())
                    end_point = np.append(end_point, point_e.copy())
        elif i == 2:
            for j in [0, 1]:
                if j == 0:
                    point_s = []
                    point_e = []
                    for k in range(4):
                        trans = np.dot(trans_link[i], trans_group3[j][k])
                        point1 = np.dot(trans, [0, 0, 0, 1])
                        point2 = np.dot(trans, [0.2, 0, 0, 1])
                        point_s.append(point1[:3].copy())
                        point_e.append(point2[:3].copy())
                    start_point = np.append(start_point, point_s.copy())
                    end_point = np.append(end_point, point_e.copy())
                else:
                    point_s = []
                    point_e = []
                    for k in range(5):
                        trans = np.dot(trans_link[i], trans_group3[j][k])
                        point1 = np.dot(trans, [0, 0, 0, 1])
                        point2 = np.dot(trans, [0.2, 0, 0, 1])
                        point_s.append(point1[:3].copy())
                        point_e.append(point2[:3].copy())
                    start_point = np.append(start_point, point_s.copy())
                    end_point = np.append(end_point, point_e.copy())

        else:
            for j in [0, 1]:
                if j == 0:
                    point_s = []
                    point_e = []
                    for k in range(6):
                        trans = np.dot(trans_link[i], trans_group4[j][k])
                        point1 = np.dot(trans, [0, 0, 0, 1])
                        point2 = np.dot(trans, [0.2, 0, 0, 1])
                        point_s.append(point1[:3].copy())
                        point_e.append(point2[:3].copy())
                    start_point = np.append(start_point, point_s.copy())
                    end_point = np.append(end_point, point_e.copy())
                else:
                    point_s = []
                    point_e = []
                    trans = np.dot(trans_link[i], trans_group4[j])
                    point1 = np.dot(trans, [0, 0, 0, 1])
                    point2 = np.dot(trans, [0.0, 0, -0.2, 1])
                    point_s.append(point1[:3].copy())
                    point_e.append(point2[:3].copy())
                    start_point = np.append(start_point, point_s.copy())
                    end_point = np.append(end_point, point_e.copy())

    start_point = start_point.reshape(35, 3)
    end_point = end_point.reshape(35, 3)
    start_points = start_point[1:]
    end_points = end_point[1:]
    ray_from_position = start_points.tolist()
    ray_to_position = end_points.tolist()

    rayMissColor = [1, 0, 0]
    rayHitColor = [0, 1, 0]
    print("type of start point:{}, type of ray_from_position:{}".format(type(start_points), type(ray_from_position)))
    # print("ray_from_list:{}".format(ray_from_position))

    for i in range(len(start_points)):
        touch.append(p.addUserDebugLine(start_points[i], end_points[i], rayMissColor, bodyId))
    return touch, start_points, end_points


    """touch = p.rayTestBatch(
        rayFromPositions=start_points,
        rayToPositions=end_points)
    touch_point = touch[3]
    for i in range(len(touch_point)):
        obj_point_x = touch_point[i][0]
        obj_point_y = touch_point[i][1]
        obj_point = np.append(obj_point, [obj_point_x, obj_point_y])

    for i in range(len(start_point)):
        p.addUserDebugLine(lineFromXYZ=start_point[i],
                           lineToXYZ=end_point[i],
                           lineColorRGB=[0.5, 0.5, 0.5],
                           parentObjectUniqueId=bodyId
                           )
    return obj_point[1:]"""