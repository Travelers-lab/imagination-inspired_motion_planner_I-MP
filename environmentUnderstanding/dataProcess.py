from os.path import join, dirname, abspath
import sys
path = join(dirname(dirname(abspath(__file__))))
sys.path.append(path)

import numpy as np
from motionPlanner.vector import Vector
from environmentUnderstanding.sensorUnify import sensorInfoUnify, hsSensorInfoUnify
from environmentUnderstanding.parameterIdentification import regression
from tactilePerception.sensorData import approachingData, contactData


# def environmentUnderstanding(robots, sensor_transform_matrix, hsSensorInfo, objectsId, objects):
#     approaching_data = approachingData(sensor_transform_matrix)
#     contact_data = contactData(robots, objectsId)
#     sensorInfo = sensorInfoUnify(approachingData=approaching_data, contactData=contact_data)
#     hsSensorInfo = hsSensorInfoUnify(sensorInfo, hsSensorInfo)
#     objects = object_understanding(sensorInfo, objects, hsSensorInfo)
#     return objects, hsSensorInfo

class EnvironmentUnderstanding:
    def __init__(self):
        return

    def implicit_infer(self, hs_sensor_info, objects):
        if len(hs_sensor_info['force']) >= 100 and objects["physics"] == 'waiting understanding':
            a1 = len(hs_sensor_info['force']) - 1
            Y = np.array(hs_sensor_info["force"][1:]).reshape(a1, 1)
            X1 = np.array(hs_sensor_info["displacement"][1:]).reshape(a1, 1)
            X2 = np.array(hs_sensor_info["velocity"][1:]).reshape(a1, 1)
            X = np.concatenate((X1, X2), axis=1)
            a = regression(X, Y)
            if  a[0]==0 or abs(a[2]) > 10:
                attribute = "fixed"
                physics = None
                # with open("data_ext.txt", "a") as file:
                #     file.write("obj_physics_infer:" + str(a))
            elif abs(a[2]) > 0 and abs(a[2]) <= 10:
                attribute = "operable"
                physics = a
            else:
                pass
        else:
            attribute = None
            physics = 'waiting understanding'
        return attribute, physics

    def object_understanding(self, sensor_info, hs_sensor_info, objects):
        for key in sensor_info:
            if key not in objects:
                objects[key] = {}
                objects[key]['center'] = Vector(sensor_info[key]['center'])
                objects[key]['states'] = sensor_info[key]["states"]
                objects[key]['radius'] = 0.05
                objects[key]['attribute'] = None
                objects[key]['physics'] = 'waiting understanding'
            else:
                objects[key]['center'] = Vector(sensor_info[key]['center'])
                if objects[key]["states"] == "contacting" or objects[key]["states"] == "approaching":
                    objects[key]['states'] = sensor_info[key]["states"]
                if objects[key]['states'] == 'contacting' and objects[key]['physics'] == 'waiting understanding':
                    objects[key]['attribute'], objects[key]['physics'] = self.implicit_infer(
                        hs_sensor_info['contacting'][key], objects[key])
                else:
                    pass
        return

    def object_representation(self, robots, sensor_transform_matrix, environment_info, objectsId,  agent_states):
        approaching_data = approachingData(sensor_transform_matrix)
        contact_data = contactData(robots, objectsId)
        sensorInfo = sensorInfoUnify(approachingData=approaching_data, agent_states=agent_states, contactData=contact_data)
        hsSensorInfoUnify(sensorInfo, environment_info['hs_sensor_info'])
        self.object_understanding(sensorInfo, environment_info['hs_sensor_info'], environment_info['objects'])
        return environment_info