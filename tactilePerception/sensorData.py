from os.path import join, dirname, abspath
import sys
path = dirname(dirname(abspath(__file__)))
sys.path.append(path)
import pybullet as p
import numpy as np

def approachingData(sensor_transform_matrix):
    sensorPosition = []
    rayEndPosition = []
    rayRange = 0.05
    armRadius = 0.042
    sideOffset = 0.105
    rayMissColor = [1, 0, 0]

    for key in sensor_transform_matrix:
        if "m9" in key or "link11sensor_num6" in key:
            sensorPosition.append(sensor_transform_matrix[key].dot([0, 0, sideOffset, 1])[:3])
            rayEndPosition.append(sensor_transform_matrix[key].dot([0, 0, sideOffset + rayRange, 1])[:3])
        else:
            sensorPosition.append(sensor_transform_matrix[key].dot([armRadius, 0, 0, 1])[:3])
            rayEndPosition.append(sensor_transform_matrix[key].dot([armRadius + rayRange, 0, 0, 1])[:3])

    approachingDataset = p.rayTestBatch(sensorPosition, rayEndPosition)
    # print("approachingDataset:{}".format(approachingDataset))
    # touch = []
    # for i in range(len(sensorPosition)):
    #     touch.append(p.addUserDebugLine(sensorPosition[i], rayEndPosition[i], rayMissColor, bodyId))
    return approachingDataset

def contactData(robots, objectsId):
    contactDataset=[]
    for i in range(len(objectsId)):
        points = p.getContactPoints(bodyA=robots,
                                   bodyB=objectsId[i])
        if len(points) == 1:
            if len(points[0]) == 14:
                contactDataset.append(points[0])
            elif len(points[0]) >= 1 and len(points[0]) <= 5:
                contactDataset.append(points[0][0])
        elif len(points) == 14:
            contactDataset.append(points)
    # print("contactData:{}".format(contactDataset))
    return contactDataset


