from os.path import join, dirname, abspath
import sys
import numpy as np
import math
path = join(dirname(dirname(abspath(__file__))))
sys.path.append(path)

from perception.leanerRegression import regression
from new_domain.Vector import Vector

objRadius = 0.05

def objUpdate(hs_sensor_info, object):
    if len(hs_sensor_info['force']) >= 100 and object["physics"] == 'waiting understanding':
        a = len(hs_sensor_info['force'])
        X = np.array(hs_sensor_info["force"]).reshape(a, 1)
        Y = np.array(hs_sensor_info["displacement"]).reshape(a, 1)
        print("X{}, Y:{}".format(X, Y))
        a, b = regression(Y, X)
        print("a:{}, b:{}".format(a, b))
        if math.fabs(a) > 100:
            attribute = "fixed"
            physics = None
        elif math.fabs(a) > 20 and math.fabs(a) <= 100:
            attribute = "elastic"
            physics = math.fabs(b)
        elif math.fabs(a) <= 20 and b > 0:
            attribute = "movable"
            physics = b
        else:
            pass
    else:
        attribute = 'contacting'
        physics = 'waiting understanding'
    return attribute, physics

def objDescroption(sensor_info, objects, hs_sensor_info):
    for key in sensor_info:
        if key not in objects:
            objects[key] = {}
            objects[key]['center'] = Vector(sensor_info[key]['center'])
            objects[key]['states'] = sensor_info[key]["states"]
            objects[key]['radius'] = objRadius
            objects[key]['attribute'] = None
            objects[key]['physics'] = 'waiting understanding'
        else:
            objects[key]['center'] = Vector(sensor_info[key]['center'])
            if objects[key]["states"] == "contacting" or objects[key]["states"] == "approaching":
                objects[key]['states'] = sensor_info[key]["states"]
            if objects[key]['states'] == 'contacting' and objects[key]['physics'] == 'waiting understanding':
                objects[key]['attribute'], objects[key]['physics'] = objUpdate(hs_sensor_info['contacting'][key], objects[key])
            else:
                pass
    return objects


