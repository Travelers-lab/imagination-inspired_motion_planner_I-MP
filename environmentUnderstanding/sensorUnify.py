import numpy as np
import pybullet as p

from motionPlanner.vector import Vector


def sensorInfoUnify(approachingData, agent_states, contactData = None):
    sensor_info = {}
    if contactData != None:
        for i in range(len(contactData)):
            if len(contactData[i]) == 0:
                pass
            elif contactData[i][2] == 1 or contactData[i][2] == 2 or contactData[i][9] == 0:
                pass
            else:
                # print("contactData[i][0][2]:{}".format(contactData[i][2]))
                key = "object" + str(contactData[i][2])
                if key not in sensor_info:
                    sensor_info[key] = {}
                    sensor_info[key]['states'] = 'contacting'
                    sensor_info[key]["center"] = list(p.getLinkState(contactData[i][2], 0)[4][:2])
                    sensor_info[key]['position'] = contactData[i][5]
                    sensor_info[key]['force'] = contactData[i][9]
                else:
                    sensor_info[key]['states'] = 'contacting'
                    sensor_info[key]["center"] = list(p.getLinkState(contactData[i][2], 0)[4][:2])
                    sensor_info[key]['position'] = contactData[i][5]
                    sensor_info[key]['force'] = contactData[i][9]
                    for i in range(len(approachingData)):
                        if "object" + str(approachingData[i][0]) == key:
                            approachingData = approachingData[:i] + approachingData[i+1:]
                        else:
                            pass
    # print("approachingData:{}".format(approachingData))
    for i in range(len(approachingData)):
        if approachingData[i][0] == -1 or approachingData[i][0] == 0 or approachingData[i][0] == 1 or approachingData[i][0] == 2:
            pass
        else:
            key = "object" + str(approachingData[i][0])
            if key not in sensor_info:
                sensor_info[key] = {}
                sensor_info[key]['states'] = 'approaching'
                # print("approachingData[i][0]:{}".format(approachingData[i][0]))
                sensor_info[key]["center"] = list(p.getLinkState(approachingData[i][0], approachingData[i][1])[4][:2])
                sensor_info[key]['position'] = approachingData[i][3]
            elif sensor_info[key]['states'] == 'contacting':
                sensor_info[key]['states'] == 'approaching'
                sensor_info[key]['position'] = approachingData[i][3]
            else:
                pass
    # print("sensorInfo:{}".format(sensorInfo))
    return sensor_info

def hsSensorInfoUnify(sensor_info, hs_sensor_info):
    object_identity_info = {}
    if 'contacting' not in hs_sensor_info:
        hs_sensor_info['contacting'] = {}
    if "approaching" not in hs_sensor_info:
        hs_sensor_info['approaching'] = []
    for key in sensor_info:
        if sensor_info[key]['states'] == 'contacting':
            if key not in hs_sensor_info['contacting']:
                hs_sensor_info['contacting'][key] = {}
                hs_sensor_info['contacting'][key]['force'] = []
                hs_sensor_info['contacting'][key]["position"] = []
                hs_sensor_info['contacting'][key]["displacement"] = [0]
                hs_sensor_info['contacting'][key]['velocity'] = [0]
                hs_sensor_info['contacting'][key]['force'].append(sensor_info[key]["force"])
                hs_sensor_info['contacting'][key]["position"].append(sensor_info[key]["center"])
            else:
                hs_sensor_info['contacting'][key]['force'].append(sensor_info[key]["force"])
                hs_sensor_info['contacting'][key]["position"].append(sensor_info[key]["center"])
                l = len(hs_sensor_info["contacting"][key]["position"])
                displacement = Vector(hs_sensor_info['contacting'][key]["position"][l-1]) - Vector(hs_sensor_info['contacting'][key]["position"][l-2])
                # d = [0]*3
                # for i in range(len(hs_sensor_info['contacting'][key]["position"][l-1])):
                #     d[i] = hs_sensor_info['contacting'][key]["position"][l-1][i] - hs_sensor_info['contacting'][key]["position"][l-2][i]
                hs_sensor_info['contacting'][key]["displacement"].append(displacement.length)
                hs_sensor_info['contacting'][key]['velocity'].append(displacement.length*120)

        elif sensor_info[key]['states'] == 'approaching':
            hs_sensor_info['approaching'].append(sensor_info[key]['position'][:2])
    return hs_sensor_info


def approaching_perception(approachingData, point_cloud_sets):
    for i in range(len(approachingData)):
        if approachingData[i][0] == -1 or approachingData[i][0] == 0 or approachingData[i][0] == 1 or approachingData[i][0] == 2:
            pass
        else:
            point_cloud_sets.append(approachingData[i][3][:2])
    # print("sensorInfo:{}".format(sensorInfo))
    return point_cloud_sets