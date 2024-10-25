from os.path import join, dirname, abspath
import sys
path = join(dirname(dirname(abspath(__file__))), "stateEstimation")
sys.path.append(path)
from localization import localization as lz
import numpy as np
import pybullet as p
from leanerRegression import regression

class DataProcess:

    def __init__(self, boxID, available_sensor_link, body_items):
        self.boxID = boxID
        self.available_sensor_link = available_sensor_link
        self.body_items = body_items#障碍物的列表
        self.touch_line, self.ray_from_point, self.ray_to_point = lz(boxID, available_sensor_link)

    def sensorInfoUnify(self):#检测并记录最新的传感器信息,循环扫描不做存储
        approximateResult = p.rayTestBatch(self.ray_from_point, self.ray_to_point)
        sensor_info = {}
        for i in self.body_items:
            contact = p.getContactPoints(self.boxID, i)
            if contact and ('object' + str(i)) not in sensor_info["contactID"]:
                sensor_info["contactID"]['object' + str(i)] = {}
                sensor_info["contactID"]['object' + str(i)]['ID'] = contact[2]
                sensor_info["contactID"]['object' + str(i)]['position'] = contact[5]
                sensor_info["contactID"]['object' + str(i)]['force'] = contact[9]
            else:
                sensor_info["contactID"]['object' + str(i)]['ID'] = contact[2]
                sensor_info["contactID"]['object' + str(i)]['position'] = contact[5]
                sensor_info["contactID"]['object' + str(i)]['force'] = contact[9]
        num_p = 0
        for i in range(len(approximateResult)):
            if approximateResult[i][0] > 0 and ('object' + str(approximateResult[i][0]) not in sensor_info["contactID"]):
                sensor_info["approachingID"]['object' + str(num_p)]['ID'] = approximateResult[i][0]
                sensor_info["approachingID"]['object' + str(num_p)]['position'] = approximateResult[i][3]
            num_p += 1
        return sensor_info

def hsSensorInfoUnify(sensor_info, hs_sensor_info):#检测并记录传感器的信息，以便于回归算法进行参数辨识
    hs_sensor_info = hs_sensor_info
    sensor_info = sensor_info
    object_identity_info = {}
    for key in sensor_info["contactID"]:
        if key in hs_sensor_info:
            hs_sensor_info[key]['force'].append(sensor_info["contactID"][key]["force"])
            hs_sensor_info[key]["position"].append(sensor_info["contactID"][key]["position"])
            i = len(hs_sensor_info[key]["position"])
            delta_displacement = hs_sensor_info[key]["position"][i] - hs_sensor_info[key]["position"][i-1]
            hs_sensor_info[key]["displacement"].append(np.linalg.norm(delta_displacement, 1))
            if len(hs_sensor_info[key]["displacement"]) == 10:
                object_identity_info[key] = {}
                object_identity_info[key]["x"] = hs_sensor_info[key]['force']
                object_identity_info[key]["y"] = hs_sensor_info[key]["displacement"]
            return hs_sensor_info
        else:
            hs_sensor_info[key] = {}
            hs_sensor_info[key]["force"] = sensor_info["contactID"][key]["force"]
            hs_sensor_info[key]["position"] = sensor_info["contactID"][key]["position"]
            hs_sensor_info[key]["displacement"] = [0]


def objectUpdate(boxID, available_sensor_link, body_items):
    sensor_information = DataProcess(boxID, available_sensor_link, body_items)
    sensor_info = sensor_information.sensorInfoUnify()
    hs_sensor_info = sensor_information.hsSensorInfoUnify()
    objects = {}

    for key in sensor_info:
        for keys in sensor_info[key]:
            if keys not in objects:
                objects[keys] = {}
                ID_center = p.getLinkState(sensor_info[key][keys]["ID"], 0)
                objects[keys]["center"] = ID_center[4]
                if key == "contactID":
                    objects[keys]["attribute"] = 'contacting'
                    objects[keys]['position'] = sensor_info[key][keys]['position']
                elif key == "approachingID":
                    objects[keys]["attribute"] = 'approaching'
                    objects[keys]['position'] = sensor_info[key][keys]['position']
                objects[keys]['radius'] = 0.1
    for key in hs_sensor_info:
        a, b = regression(hs_sensor_info[key]["x"], hs_sensor_info[key]["y"])
        if a > 100:
            objects[key]["attribute"] = "fixed"
        elif a >10 and a <= 100 :
            objects[key]["attribute"] = "elastic"
            objects[key]["physics"] = a
        elif a <= 10 and b > 0:
            objects[key]["attribute"] = "movable"
            objects[key]["physics"] = b
    judgement = {}
    for key in objects:
        if objects[key]['attribute'] == "contacting":
            judgement["contactID"] = key
    return objects, judgement