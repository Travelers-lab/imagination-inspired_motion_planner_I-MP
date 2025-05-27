from os.path import join, dirname, abspath
import sys

path = join(dirname(dirname(abspath(__file__))))
sys.path.append(path)

from motionPlanner.environmentRepresentation import EnergyState
from motionPlanner.domain import Domain
from motionPlanner.vector import Vector
import math


class MotionImagination:

    def __init__(self, cfg):
        # cfg: Configuration object containing parameters for the motion imagination system
        #   - effect: Influence range parameter
        #   - facter[k_att]: Attraction force coefficient
        #   - workspace_dumping: Damping coefficient for workspace boundaries
        #   - object_dumping: Damping coefficient for object interactions
        #   - fixed_object_k_rep: Repulsion coefficient for fixed objects
        #   - workspace: Workspace boundaries or constraints
        self.effect = cfg.effect
        self.environmentEnergy = EnergyState(cfg.facter[cfg.k_att], cfg.workspace_dumping, cfg.object_dumping, cfg.fixed_object_k_rep)
        self.domain = Domain(cfg.effect, cfg.workspace, cfg.fixed_object_k_rep, cfg.object_dumping)

    def action_imagination(self, agent_state, target_point, objects):
        # agent_state: Dictionary containing agent's current state (position, velocity, etc.)
        # target_point: Desired target position as Vector or list
        # objects: Dictionary containing all objects in environment
        g_star = self.domain.imagined_state(agent_state, target_point)
        update_object = self.domain.object_topology(agent_state, objects, g_star, target_point)
        return g_star, update_object

    def perception_motion_coordination(self, objects):
        # objects: Dictionary containing all objects in environment and their states
        mission = 'motion task'
        for key in objects:
            if objects[key]['states'] == 'contacting' and objects[key]['attribute'] == None:
                mission = 'active perception'
            else:
                pass
        return mission

    def solving_gradient(self, objects, f_active, t, agent_state, target_point):
        # objects: Dictionary containing all objects in environment
        # f_active: Active force vector (used in perception mode)
        # t: Current time step or timestamp
        # agent_state: Dictionary containing agent's current state
        # target_point: Desired target position as Vector or list
        g_star, update_object = self.action_imagination(agent_state, target_point, objects)
        mission = self.perception_motion_coordination(objects)
        if mission == "motion task":
            f_att = self.environmentEnergy.attraction_force(agent_state, g_star)
            f_rep, f_interact = self.environmentEnergy.repulsion_force(agent_state, update_object, g_star)
            total_force = f_att + f_rep
        else:
            f_interact = self.environmentEnergy.active_force(agent_state=agent_state, objects=objects, t=t, g_star=g_star)
            t += 1
            if t >= 100:
                t = 0
            a = [0] * len([1, 2])
            f_rep = Vector(a)
            f_att = Vector(a)
            total_force = f_interact
        planed_force = []
        for i in range(2):
            planed_force.append(total_force.vec_p[i])


        return planed_force, t, f_rep, f_att, f_interact



if __name__ == "__main__":
    p1 = Vector([1, 1, 1])
    p2 = Vector([1.5, 1, 1])
    effect = 0.2
    agent_pos = [0.8041508793830872, 0.3668517470359802, 1.0503519773483276]
    agent_vel = [0, 0, 0]
    workspace = [[0, 0], [100, 100]]
    target_point = Vector([0.8, -0.2, 1.0503])
    # object = {'obj_1': {"center": p1, "attribute": "approaching", "radius": 1},
    #           'obj_2': {"center": p2, "attribute": "approaching", "radius": 2}}
    object ={}
    f = motionPlanning(effect=effect, agent_pose=agent_pos, agent_vel=agent_vel,
                       workspace=workspace, target_point=target_point, objects=object)
    print(f)