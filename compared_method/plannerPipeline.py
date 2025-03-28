from compared_method.model_basedPlanner.APFPlanner import ArtificialPotentialField as apf_planner
from compared_method.model_basedPlanner.mapTopology import PointCloudToGridMap as apf_pcg
from compared_method.model_basedPlanner.trajectoryOptimization import SmoothInterpolation

from compared_method.sampler_basedPlanner.map_generator import PointCloudToGridMap as sab_pcg
from compared_method.sampler_basedPlanner.PRM_planner import PRMPlanner as prm

from compared_method.simulation_basedPlanner.controlPointGenerate import ControlPointGenerator
from compared_method.simulation_basedPlanner.B_splinePlanner import BSplinePathPlanner

from environmentUnderstanding.dataProcess import EnvironmentUnderstanding
from motionPlanner.motionPlanning import MotionImagination

class ModeledPipeline:
    def __init__(self, cfg):
        self.map_generator = apf_pcg(space=cfg.space_range, resolution=cfg.resolution)
        self.apf_planner = apf_planner(x_bounds=cfg.x_bound, y_bounds=cfg.y_bounds, k=cfg.k, kr=cfg.ky, p=cfg.p)
        self.optimizer = SmoothInterpolation()

    def forward(self, environment_info, robot_pos, goal):
        grid = self.map_generator.convert(environment_info['point_cloud_sets'])
        path_node = self.apf_planner.find_path_nodes(grid, robot_pos[:2], goal)
        if len(path_node) == 1:
            trajectory = []
            pass
        else:
            trajectory = self.optimizer.forward(path_node)
        return [(x, y, 1.14345) for x, y in trajectory]

class SampledPipeline:
    def __init__(self, cfg):
        self.map_generator = sab_pcg(space=cfg.space_range, resolution=cfg.resolution)
        self.prm_planner = prm(x_bounds=cfg.x_bound, y_bounds=cfg.y_bounds,
                               num_samples=cfg.num_samples, connection_radius=cfg.connection_radius)

    def forward(self,  environment_info, start, goal):
        grid_map = self.map_generator.convert(environment_info['point_cloud_sets'])
        trajectory = self.prm_planner.plan(grid_map, start[:2], goal)
        return trajectory

class SimulatedPipeline:
    def __init__(self, cfg):
        self.cfg = cfg
        self.control_point_generator = ControlPointGenerator(base_position=cfg.base_position)
        self.b_spline_planner = BSplinePathPlanner()

    def forward(self,  environment_info, start_point, goal, queue_traj, queue_result, queue_obstacles):
        control_point, object_coord = self.control_point_generator.update_control_points(start_point, goal,
                                                                                         environment_info['point_cloud_sets'])
        trajectory = self.b_spline_planner.generate_path(control_point, self.cfg.num_points)
        queue_obstacles.put(object_coord)
        queue_traj.put(trajectory)
        path = queue_result.get()
        return path

class ImagedPipeline:
    def __init__(self, cfg):
        self.cfg = cfg
        self.environment_understander = EnvironmentUnderstanding()
        self.motion_imaginator = MotionImagination(cfg)

    def forward(self, robot, environment_info, agent_state, f_active, target_point, t):
        environment_info = self.environment_understander.object_representation(robot.robot, robot.transformMatrix(self.cfg.body_link), environment_info, robot.object, agent_state)
        f_total, t, f_rep, f_att, f_interact = self.motion_imaginator.solving_gradient(objects=environment_info['objects'], f_active=f_active, agent_state=agent_state, target_point=target_point, t=t)
        return f_total, t

