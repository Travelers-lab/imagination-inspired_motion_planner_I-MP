from os.path import join, dirname, abspath
import sys
import numpy as np
path = join(dirname(dirname(abspath(__file__))))
sys.path.append(path)

from motionPlanner.domain import *
from motionPlanner.vector import Vector

class EnergyState:

    def __init__(self, k_att, workspace_dumping, object_dumping, fixed_object_k_rep):
        # k_att: Attraction force coefficient
        # workspace_dumping: Damping coefficient for workspace boundaries
        # object_dumping: Damping coefficient for object interactions
        # fixed_object_k_rep: Repulsion coefficient for fixed objects
        self.k_att = k_att
        self.workspace_dumping = workspace_dumping
        self.object_dumping = object_dumping
        self.fixed_object_k_rep = fixed_object_k_rep

    def attraction_force(self, agent_states, g_star):
        # agent_states: Dictionary containing agent's state (position, velocity, etc.)
        # g_star: Desired direction vector (from imaged state calculation)
        if g_star.length > 0.001:
            f_att = g_star * self.k_att + agent_states['vel'] * self.workspace_dumping
        else:
            f_att = g_star * 0
        return f_att

    def repulsion_force(self, agent_state, object, g_star):
        # agent_state: Dictionary containing agent's current state
        # object: Dictionary containing all objects in environment
        # g_star: Desired direction vector (from imaged state calculation)
        a = [0] * len(agent_state['pos'].vec_p)
        f_rep_total = Vector(a)
        interacting_force = Vector(a)
        if g_star.length < 0.005:
            pass
        else:
            for key in object:
                f_rep_local = Vector(a)
                if object[key]['states'] == 'approaching':
                    if object[key]['attribute'] == None:
                        component_scale = agent_state['vel'].dot(object[key]['direction']) / object[key]['direction'].length ** 2
                        f_rep_local = (object[key]['direction'] * component_scale) * object[key]['energy_param']
                    elif object[key]['attribute'] == 'operable':
                        f_rep_local = object[key]['direction'] * object[key]['energy_param']
                    elif object[key]['attribute'] == 'fixed':
                        f_rep_local = object[key]['direction'] * object[key]['energy_param']
                elif object[key]['states'] == 'contacting':
                    if object[key]['attribute'] == None:
                        f_rep_local = Vector([0, 0])
                    elif object[key]['attribute'] == 'operable':
                        f_rep_local = object[key]['direction'] * object[key]['energy_param'][2]/500
                        interacting_force = Vector([0, 0])
                    elif object[key]['attribute'] == 'fixed':
                        f_rep_local = object[key]['center'] * self.fixed_object_k_rep / 10
                    else:
                        f_rep_local = Vector([0, 0])
                f_rep_total += f_rep_local
        return f_rep_total, interacting_force

    def repulsion_force_2(self, agent_state, object, g_star):
        # agent_state: Dictionary containing agent's current state
        # object: Dictionary containing all objects in environment
        # g_star: Desired direction vector (from imaged state calculation)
        a = [0] * len(agent_state['pos'].vec_p)
        f_rep_total = Vector(a)
        interacting_force = Vector(a)
        if g_star.length < 0.005:
            pass
        else:
            for key in object:
                for i in object[key]['energy_param']:
                    f_rep_local = object[key]['direction'] * (-i)
                    f_rep_total += f_rep_local
        return f_rep_total, interacting_force


    def active_force(self, agent_state, objects, t, g_star):
        # agent_state: Dictionary containing agent's current state
        # objects: Dictionary containing all objects in environment
        # t: Current time step or timestamp
        # g_star: Desired direction vector (from imaged state calculation)
        offset = 0.4
        omega = 0.5 * math.pi / 100
        direction = g_star
        f_active = Vector(direction.direction) * offset * math.sin(omega * t)
        return f_active


if __name__ == "__main__":
    p1 = Vector([2, 5])
    p2 = Vector([5, 1])
    k_att = 1
    workspace_dumping = -2
    object_dumping = -3
    fixed_object_k_rep = 4
    effect = 0.2
    agent_state['vel'] = Vector([20, 20])
    object = {'obj_1': {"center":  p1, "attribute": "approaching", "radius": 1}, 'obj_2': {"center": p2, "attribute": "approaching", "radius": 2}}
    goal = Vector([15, 15])
    agent_state = Vector([2, 1])
    workspace = [[0, 0], [20, 20]]
    domain = Domain(effect, agent_state, workspace)
    target_point = domain.TargetPoint(goal)
    print("target_point:{}".format(target_point))
    object_update = domain.ObjectPoint(object)
    apf = eneragyState(k_att, workspace_dumping, object_dumping, fixed_object_k_rep, effect, agent_state,  agent_state['vel'], workspace)
    att = apf.AttractionForce(target_point)
    rep = apf.RepulFource(object_update)
    print(att.vec_p)
    print(rep.vec_p)