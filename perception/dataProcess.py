from os.path import join, dirname, abspath
import sys

path = join(dirname(dirname(abspath(__file__))))
sys.path.append(path)

from lunch.perception.sensorUnify import sensorInfoUnify, hsSensorInfoUnify
from lunch.perception.objDescription import *
from lunch.stateEstimation.sensorData import approachingData, contactData


def dataProcess(robots, sensor_transform_matrix, hsSensorInfo, objectsId, objects):
    approaching_data = approachingData(sensor_transform_matrix, robots)
    contact_data = contactData(robots, objectsId)
    sensorInfo = sensorInfoUnify(approachingData=approaching_data, contactData=contact_data)
    hsSensorInfo = hsSensorInfoUnify(sensorInfo, hsSensorInfo)
    objects = objDescroption(sensorInfo, objects, hsSensorInfo)
    return objects, hsSensorInfo

