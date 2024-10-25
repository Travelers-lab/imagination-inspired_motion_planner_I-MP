import pybullet as p
import numpy as np
import math

class DataProcess:
    def __init__(self, bodyID, body_links, arm_radius):
        self.bodyID = bodyID
        self.body_links = body_links
        self.arm_radius = arm_radius
        self.sensor_transform_matrix = {}
        self.sensor_position = {}

    def transform_matrix(self):
        joint_trans = np.eye(4)
        trans_matrix = np.eye(4)
        trans_link = []
        sensor_transform_matrix = {}
        sensor_position = {}
        for i in self.body_links:
            joint_state = p.getLinkState(bodyUniqueId=self.bodyId, linkIndex=i)
            joint_pos, joint_ori = joint_state[4], joint_state[5]
            rot_matrix = p.getMatrixFromQuaternion(joint_ori)
            joint_trans[:3, :3] = np.array(rot_matrix).reshape(3, 3)
            joint_trans[:3, 3] = np.array(joint_pos)
            trans_link.append(joint_trans.copy())
        sensor_translation_param = [[0.0455, 0.0], [0.0185, -0.0265], [0.0185, -0.0265], [-0.0185, -0.057]]
        sensor_rotation_matrix = {"body_link0sensor_line0": [1.701748, 2.6626, 3.62156, 4.58149], "body_link0sensor_line1": [1.22179, 2.18169, 3.14159, 4.10149, 5.06139], "body_link1sensor_line0": [1.701748, 2.6626, 3.62156, 4.58149], "body_link1sensor_line1": [1.22179, 2.18169, 3.14159, 4.10149, 5.06139], "body_link2sensor_line0": [1.701748, 2.6626, 3.62156, 4.58149], "body_link2sensor_line1": [1.22179, 2.18169, 3.14159, 4.10149, 5.06139], "body_link3sensor_line0": [0.0, 1.04719, 2.0944, 3.14159, 4.18879, 5.235987], "body_link3sensor_line1": [0]}
        for i in range(len(self.body_links)):
            sensor_num = 0
            for j in sensor_translation_param[i]:
                for k in sensor_rotation_matrix["body_link"+str(i)+"sensor_line"+str(j)]:
                    first_line = [0.0, 0.0, j]
                    rot_matrix1 = p.getMatrixFromQuaternion([0.0, 0.0, math.sin(k / 2), math.cos(k / 2)])
                    trans_matrix[:3, :3] = np.array(rot_matrix1).reshape(3, 3)
                    trans_matrix[:3, 3] = np.array(first_line)
                    sensor_transform_matrix["joint" + str(self.body_links[i]) + 'sensor_num' + str(sensor_num)] = np.dot(trans_matrix[i], trans_matrix)
                    if i == 3 and j == 1:
                        point = [0, 0, sensor_translation_param[i][j], 1]
                    else:
                        point = [self.arm_radius, 0, 0, 1]
                    sensor_position['body_link' + str(self.body_links[i]) + 'sensor_num' + str(sensor_num)] = np.dot(sensor_transform_matrix["joint" + str(self.body_links[i]) + 'sensor_num' + str(sensor_num)], point)
                    sensor_num += 1
        return sensor_transform_matrix, sensor_position

    def approachingInfo(self):

        approachingInfo = p.rayTestBatch()


    def get_contact_info(self, sensor_data):
        #sensor position
        sensor_matrix, sensor_position = DataProcess.transform_matrix()
        active_sensor = {"approachingID": {}, "contactID": {}}
        for key in sensor_data:
            if sensor_data[key][0] > 0:
                active_sensor['approachID'][key] = np.dot(sensor_position[key], [sensor_data[key][0], 0, 0, 1])
            if sensor_data[key][1] > 0:
                active_sensor['contactID'][key]["position"] = sensor_position[key]
                active_sensor['contactID'][key]["force"] = sensor_data[key][1]
        return active_sensor

    def hs_contact_unify(self, sensor_data):
        hs_tactile_info = {}
        active_sensor = DataProcess.get_contact_info(sensor_data)
        object_identity_info = {}
        for key in active_sensor["contactID"]:# the module of tactile
            if key in hs_tactile_info:
                hs_tactile_info[key]['force'].append(active_sensor['contactID'][key]["force"])
                hs_tactile_info[key]['position'].append(active_sensor['contactID'][key]["position"])
                a = len(hs_tactile_info[key]["deformation"])
                delta_deformation = np.linalg.norm(hs_tactile_info[key]['position'][a] - hs_tactile_info[key]['position'][a-1])
                hs_tactile_info[key]["deformation"].append(delta_deformation)
                if len(hs_tactile_info[key]["deformation"]) == 10:
                    object_identity_info["x"] = hs_tactile_info[key]['force']
                    object_identity_info["y"] = hs_tactile_info[key]["deformation"]
                    return object_identity_info
            else:
                hs_tactile_info[key] = {}
                hs_tactile_info[key]["deformation"] = [0]
                hs_tactile_info[key]['force'] = active_sensor['contactID'][key]["force"]
                hs_tactile_info[key]['position'] = active_sensor['contactID'][key]["position"]






def main():
    with open("../cell_data.txt", "r", encoding="utf-8") as f:
        j = 0
        sensor_data = {}
        for li in f.readlines():
            li = li.strip(" \n")
            ls = list(li.split(" "))
            data = [int(x, 16) for x in ls]
            data_use = np.zeros([1, 2])
            for i in range(9):
                data_use[0][0] = ((data[i*19+1]*256 + data[i*19+2])/16384 - 0.2)*25.0/0.6
                data_approximate = data[i*19+3]*256 + data[i*19+4]
                data_use[0][1] = 25.22*(math.log10(data_approximate)) - 5.478
                sensor_data['joint' + str(j) + 'sensor_num' + str(i)] = data_use
            j += 1

        print(sensor_data)

if __name__ == "__main__":
    main()