import time
import csv
import json
import pybullet as p
import pybullet_data as pd
from omegaconf import OmegaConf
from os.path import join, dirname, abspath

from motionPlanner.vector import Vector
from robotEnvironment.robotEnvironment_stressTesting import Robot
from compared_method.plannerPipeline import ImagedPipeline
from tactilePerception.sensorData import contactData
from utils.utils import evaluate_update, check_time_difference, write_stress_test_to_csv, write_sensory_failure_data_to_csv

from environmentUnderstanding.dataProcess import EnvironmentUnderstanding
from motionPlanner.motionPlanning import MotionImagination

timeStep = 1 / 240


def load_config():
    cfg = OmegaConf.load(join(dirname(abspath(__file__)), 'config/planningConfig.json'))
    return cfg

def set_environment():
    physicalID = p.connect(p.GUI)
    p.setAdditionalSearchPath(pd.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(timeStep)
    robot_urdf = join(dirname(abspath(__file__)), 'envDescription/single_arm/left_arm.urdf')
    robot = Robot(bullet_client=p, path=robot_urdf, clint_id=physicalID)
    return robot, physicalID


def variable_reset():
    environment_info = {"point_cloud_sets": [], "hs_sensor_info": {}, "objects": {}}
    task_progress = 0
    return environment_info, task_progress


def planner_test(cfg, mission_condition):
    robot, client1 = set_environment()

    path_lengths = [0.313, 0.605, 0.855]
    target_points = [[0.63, 0.55], [0.65, 0.1]]
    target_sequence = 0

    # load planner
    perception = EnvironmentUnderstanding()
    imaged_planner = MotionImagination(cfg.imaged)

    # set simulation
    f_active = Vector([0, 0])
    t = 0
    lines = 0
    simulation_time = 0


    coordinates = [[0.63, 0.35, int(mission_condition[-1])]]
    fixed_prob = 0
    target_sequence = 0

    robot.init_environment()
    robot.load_environment(coordinates)
    environment_info, task_progress = variable_reset()
    target_point = Vector(target_points[target_sequence])
    set_planning = True
    while set_planning:
        agent_state = robot.get_effector_states()
        if (target_point - agent_state['pos']).length <= 0.005:
            target_sequence += 1
            print(f'target_sequence:{target_sequence}')
            if target_sequence == len(target_points):
                print("complete the task!")
                break
            else:
                target_point = Vector(target_points[target_sequence])
        environment_info = perception.object_representation(robot.robot, robot.transformMatrix(
            cfg.imaged.body_link), environment_info, robot.object, agent_state)
        if "Proximity_faulty" in mission_condition:
            environment_info['hs_sensor_info']['approaching'] = []
        elif  "Force_faulty" in mission_condition:
            for key in environment_info['hs_sensor_info']['contacting']:
                environment_info['hs_sensor_info']['contacting'][key]['force'] = [0 for _ in environment_info['hs_sensor_info']['contacting'][key]['force']]
        f_total, t, f_rep, f_att, f_interact = imaged_planner.solving_gradient(
            objects=environment_info['objects'], f_active=f_active, agent_state=agent_state, target_point=target_point,
            t=t)
        robot.torque_control_step(f_total)
        if target_sequence == 1:
            contact_data = contactData(robot.robot, robot.object)
            if len(contact_data) == 0:
                contact_force= 0
            else:
                contact_force = contact_data[0][9]
                # print(f"contact_force:{contact_force}")
            write_sensory_failure_data_to_csv(mission_condition, simulation_time, agent_state["vel"].vec_p, contact_force)
            simulation_time += timeStep
        p.stepSimulation(physicsClientId=client1)
        time.sleep(timeStep)
    p.disconnect(physicsClientId=client1)


if __name__ == "__main__":
    try:
        cfg = load_config()
        mission_conditions = ["None_faulty_operable_0", "None_faulty_inoperable_1",
                              "Force_faulty_operable_0", "Force_faulty_inoperable_1",
                              "Proximity_faulty_operable_0", "Proximity_faulty_inoperable_1"]
        for mission_condition in mission_conditions:
            planner_test(cfg, mission_condition)
    except KeyboardInterrupt:
        pass