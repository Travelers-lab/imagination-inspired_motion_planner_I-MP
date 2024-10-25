import numpy as np
import pybullet as p

def sensorInfoUnify(approachingData, contactData = None):
    sensorInfo = {}
    if contactData != None:
        for i in range(len(contactData)):
            if len(contactData[i]) == 0:
                pass
            elif contactData[i][2] == 1 or contactData[i][2] == 2 or contactData[i][9] == 0:
                pass
            else:
                # print("contactData[i][0][2]:{}".format(contactData[i][2]))
                key = "object" + str(contactData[i][2])
                if key not in sensorInfo:
                    sensorInfo[key] = {}
                    sensorInfo[key]['states'] = 'contacting'
                    sensorInfo[key]["center"] = list(p.getLinkState(contactData[i][2], 0)[4][:2])
                    sensorInfo[key]['position'] = contactData[i][5]
                    sensorInfo[key]['force'] = contactData[i][9]
                else:
                    sensorInfo[key]['states'] = 'contacting'
                    sensorInfo[key]["center"] = list(p.getLinkState(contactData[i][2], 0)[4][:2])
                    sensorInfo[key]['position'] = contactData[i][5]
                    sensorInfo[key]['force'] = contactData[i][9]
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
            if key not in sensorInfo:
                sensorInfo[key] = {}
                sensorInfo[key]['states'] = 'approaching'
                # print("approachingData[i][0]:{}".format(approachingData[i][0]))
                sensorInfo[key]["center"] = list(p.getLinkState(approachingData[i][0], approachingData[i][1])[4][:2])
                sensorInfo[key]['position'] = approachingData[i][3]
            elif sensorInfo[key]['states'] == 'contacting':
                sensorInfo[key]['states'] == 'approaching'
                sensorInfo[key]['position'] = approachingData[i][3]
            else:
                pass
    # print("sensorInfo:{}".format(sensorInfo))
    return sensorInfo

def hsSensorInfoUnify(sensor_info, hs_sensor_info):
    hs_sensor_info = hs_sensor_info
    sensor_info = sensor_info
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
                hs_sensor_info['contacting'][key]['delta_pose'] = [[0, 0]]
                hs_sensor_info['contacting'][key]['force'].append(sensor_info[key]["force"])
                hs_sensor_info['contacting'][key]["position"].append(sensor_info[key]["position"])
            else:
                hs_sensor_info['contacting'][key]['force'].append(sensor_info[key]["force"])
                hs_sensor_info['contacting'][key]["position"].append(sensor_info[key]["position"])
                l = len(hs_sensor_info["contacting"][key]["position"])
                d = [0]*3
                for i in range(len(hs_sensor_info['contacting'][key]["position"][l-1])):
                    d[i] = hs_sensor_info['contacting'][key]["position"][l-1][i] - hs_sensor_info['contacting'][key]["position"][l-2][i]
                hs_sensor_info['contacting'][key]["displacement"].append(np.linalg.norm(d, 1))
                hs_sensor_info['contacting'][key]['delta_pose'].append(d)

        elif sensor_info[key]['states'] == 'approaching':
            hs_sensor_info['approaching'].append(sensor_info[key]['position'][:2])
    # if 'object5' in  hs_sensor_info['contacting']:
    # print("hs sensor info approaching:{}".format(hs_sensor_info["approaching"]))
    return hs_sensor_info


