from tactilePerception.sensorData import approachingData
from environmentUnderstanding.sensorUnify import approaching_perception


def tactile_collection(sensor_transform_matrix, environment_info):
    approaching_data = approachingData(sensor_transform_matrix)
    environment_info["point_cloud_sets"] = approaching_perception(approaching_data, environment_info["point_cloud_sets"])
    return environment_info