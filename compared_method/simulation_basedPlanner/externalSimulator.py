import numpy as np
import pybullet as p
import pybullet_data
import time
from os.path import join, dirname, abspath

class PathExecutionValidator:
    def __init__(self, robot_urdf_path):
        """
        Initialize the PyBullet simulation environment, open the GUI, and load the robot.

        :param robot_urdf_path: The path to the URDF file of the robot.
        """
        # Connect to the PyBullet physics server with GUI mode
        self.physicsClient = p.connect(p.DIRECT)
        # Set the search path for PyBullet data
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setPhysicsEngineParameter(numSolverIterations=50)
        # Set the gravity in the simulation environment
        p.setGravity(0, 0, -9.81)
        # Load the plane into the simulation
        self.planeId = p.loadURDF("plane.urdf")
        # Load the robot into the simulation
        self.robotId = p.loadURDF(join(dirname(dirname(dirname(abspath(__file__)))), robot_urdf_path), [0, 0, 0],
                                  p.getQuaternionFromEuler([0, 0, 0]), physicsClientId=self.physicsClient)
        p.loadURDF(fileName=join(dirname(dirname(dirname(abspath(__file__)))), 'envDescription/simulation_table/table.urdf'),
                                    basePosition=[0.9, 0.3, 0.0],
                                    baseOrientation=[0, 0, 0, 1],
                                    useFixedBase=1, physicsClientId=self.physicsClient)
        # Initialize a list to store obstacle IDs
        self.obstacleIds = []

        self.available_joint_indexes = [i for i in range(p.getNumJoints(self.robotId)) if
                                        p.getJointInfo(self.robotId, i)[
                                            2] != p.JOINT_FIXED]
        self.robotEndEffectorIndex = 11
        self.obstacle_coord = np.array([])
        rp = np.array([-1.5708, 2.5, 0, 0, 0, 0.0, 0., 0.0, 0, 0, 0, 0, 0, 0])
        for joint_Index in range(len(self.available_joint_indexes)):
            p.resetJointState(self.robotId, self.available_joint_indexes[joint_Index],
                              rp[joint_Index])

    def _load_test_environment(self, obstacle_coordinates, trajectory):
        """
        Load obstacles into the simulation environment based on the given 3D coordinates.

        :param obstacle_coordinates: A list of 3D coordinates where obstacles will be placed.
        """

        self._reset_robot(trajectory)
        for coord in obstacle_coordinates:
            if self.obstacle_coord.size == 0:
                self.obstacle_coord = np.array([coord])
                # Load a box as an obstacle and add its ID to the list
                obstacle_id = p.loadURDF(fileName=join(dirname(dirname(dirname(abspath(__file__)))),
                                                       f'envDescription/simulation_table/cylinder{1%6}.urdf'),
                                         basePosition=coord,
                                         baseOrientation=[0, 0, 0, 1],
                                         flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
                                         useFixedBase=0, physicsClientId=self.physicsClient)
                self.obstacleIds.append(obstacle_id)
            else:
                distances = np.linalg.norm(self.obstacle_coord - np.array(coord), axis=1)
                exists = np.any(distances > 0.05)
                if exists == True:
                    self.obstacle_coord = np.vstack((self.obstacle_coord, coord))
                    # Load a box as an obstacle and add its ID to the list
                    obstacle_id = p.loadURDF(fileName=join(dirname(dirname(dirname(abspath(__file__)))), f'envDescription/simulation_table/cylinder{1}.urdf'),
                                            basePosition=coord,
                                            baseOrientation=[0, 0, 0, 1],
                                            flags=p.URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
                                            useFixedBase=0, physicsClientId=self.physicsClient)
                    self.obstacleIds.append(obstacle_id)

    def _detect_collision_force(self):
        """
        Detect the collision force between the robot and each object in the simulation.
        Return False if the collision force is greater than 5N, otherwise return True.

        :return: A boolean indicating whether the collision force is within the limit.
        """
        for obstacle_id in self.obstacleIds:
            # Get the contact points between the robot and the obstacle
            contact_points = p.getContactPoints(self.robotId, obstacle_id)
            for contact in contact_points:
                # Extract the normal force from the contact point information
                normal_force = contact[9]
                if normal_force > 5:
                    return False
        return True

    def _reset_robot(self, trajectory):
        jointPoses = p.calculateInverseKinematics(self.robotId, self.robotEndEffectorIndex,
                                                  trajectory[0])
        for joint_Index in range(len(self.available_joint_indexes)):
            p.resetJointState(self.robotId, self.available_joint_indexes[joint_Index],
                                               jointPoses[joint_Index])
    def reset_environment(self):
        for body_id in self.obstacleIds:
            p.removeBody(body_id)
        self.obstacleIds.clear()
        self.obstacle_coord = np.array([])
        rp = np.array([-1.5708, 2.5, 0, 0, 0, 0.0, 0., 0.0, 0, 0, 0, 0, 0, 0])
        for joint_Index in range(len(self.available_joint_indexes)):
            p.resetJointState(self.robotId, self.available_joint_indexes[joint_Index],
                                               rp[joint_Index])

    def _execute_trajectory(self, trajectory):
        """
        Execute the given trajectory and check the collision force at each step.
        Return False if a collision force exceeds the limit, otherwise return True when the end is reached.

        :param trajectory: A list of 3D coordinates representing the trajectory to be followed.
        :return: A boolean indicating whether the trajectory execution was successful.
        """
        for point in trajectory:
            # Move the robot to the next point in the trajectory
            jointPoses = p.calculateInverseKinematics(self.robotId, self.robotEndEffectorIndex,
                                                                   point)
            for i in range(len(self.available_joint_indexes)):
                p.setJointMotorControl2(self.robotId, self.available_joint_indexes[i],
                                                         p.POSITION_CONTROL, jointPoses[i])
            # Step the simulation forward
            p.stepSimulation(physicsClientId=self.physicsClient)
            time.sleep(1/120)
            # Check the collision force
            if not self._detect_collision_force():
                return np.array([])
        return trajectory

    def validate_path(self, obstacle_coordinates, trajectory):
        """
        Validate the given path by loading the test environment and executing the trajectory.

        :param obstacle_coordinates: A list of 3D coordinates where obstacles will be placed.
        :param trajectory: A list of 3D coordinates representing the trajectory to be followed.
        :return: A boolean indicating whether the path execution was successful.
        """
        # Load the test environment with obstacles
        self._load_test_environment(obstacle_coordinates, trajectory)
        # Execute the trajectory and return the result
        return self._execute_trajectory(trajectory)

    def disconnect(self):
        p.disconnect(physicsClientId=self.physicsClient)

    # Example usage


if __name__ == "__main__":
    # Specify the path to the robot's URDF file
    robot_urdf_path = join(dirname(dirname(dirname(abspath(__file__)))), 'envDescription/single_arm/left_arm.urdf')
    # Create an instance of the PathExecutionValidator class
    validator = PathExecutionValidator(robot_urdf_path)
    # Define the coordinates of obstacles
    obstacle_coordinates = [[1, 1, 0], [2, 2, 0]]
    # Define the trajectory for the robot
    trajectory = [[0, 0, 0], [1, 0, 0], [2, 0, 0]]
    # Validate the path and print the result
    result = validator.validate_path(obstacle_coordinates, trajectory)
    print("Path validation result:", result)
    # Disconnect from the PyBullet physics server
    p.disconnect()

