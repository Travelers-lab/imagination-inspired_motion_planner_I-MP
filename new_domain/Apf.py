from os.path import join, dirname, abspath
import sys
import numpy as np
path = join(dirname(dirname(abspath(__file__))))
sys.path.append(path)

from lunch.new_domain.Domain import *

class Apf:

    def __init__(self, k_att, workspace_dumping, object_dumping, fixed_object_k_rep, effect, agent_pose, agent_vel, workspace, g_star):
        self.k_att = k_att
        self.workspace_dumping = workspace_dumping
        self.object_dumping = object_dumping
        self.fixed_object_k_rep = fixed_object_k_rep
        if isinstance(agent_vel, list):
            self.agent_vel = Vector(agent_vel)
        if isinstance(agent_pose, list):
            self.agent_pose = Vector(agent_pose)
        self.domain = Domain(effect, agent_pose, workspace)
        self.g_star = g_star

    def AttractionForce(self):
        if self.g_star.length > 0.005:
            f_att = self.g_star * self.k_att + self.agent_vel * self.workspace_dumping
        else:
            f_att = self.g_star * 0
        return f_att

    def RepulFource(self, object):
        a = [0]*len(self.agent_pose.vec_p)
        f_rep_total = Vector(a)
        if self.g_star.length <= 0.005:
            pass
        else:
            for key in object:
                agent_to_object = object[key]['center'] - self.agent_pose
                f_rep_local = Vector(a)
                if object[key]['states'] == 'contacting' and object[key]['attribute'] == None:
                    pass
                elif object[key]['states'] == 'approaching' and object[key]['attribute'] == None:
                    print("callback 1")
                    scalar_product = 0
                    for i in range(len(object[key]['direction'].vec_p)):
                        scalar_product += self.agent_vel.vec_p[i] * agent_to_object.vec_p[i]
                    if self.agent_vel.length != 0 and agent_to_object.length <= 0.15:
                        cos_theta_i = scalar_product/(agent_to_object.length * self.agent_vel.length)
                        delta = cos_theta_i * self.agent_vel.length
                        agent_to_object_vel = Vector(agent_to_object.direction) * delta
                        f_rep_local = agent_to_object_vel * self.object_dumping
                    else:
                        f_rep_local = Vector([0]*len(self.agent_pose.vec_p))
                elif object[key]['states'] == 'approaching' and object[key]['attribute'] == 'fixed':
                    print("callback 2")
                    # print("object[key]['direction']:{}".format(object[key]['direction'].vec_p))
                    f_rep_local = object[key]['direction'] * self.fixed_object_k_rep
                    print("f_rep_local:{}".format(f_rep_local.vec_p))
                elif object[key]['states'] == 'approaching' and object[key]['attribute'] == 'movable':
                    force = [0]*len(self.agent_pose.vec_p)
                    f_rep_local = Vector(force)
                elif object[key]['states'] == 'contacting' and object[key]['attribute'] == 'movable':
                    print("callback 4")
                    if object[key]['physics'] <= 0.1 and self.agent_vel.length < 0.05:
                        force = np.array([0.01]) * agent_to_object.direction
                    else:
                        force = object[key]['physics'] * agent_to_object.direction
                    f_rep_local = Vector(force)
                elif object[key]['states'] == 'approaching' and object[key]['attribute'] == 'elastic':
                    print("callback 5")
                    f_rep_local = Vector([0]*len(self.agent_pose.vec_p))
                elif object[key]['states'] == 'contacting' and object[key]['attribute'] == 'elastic':
                    displacement = object[key]['center'] - self.agent_pose
                    # print("displacement,:{}, type:{}".format(displacement.vec_p, type(displacement.vec_p)))
                    f_rep_local = displacement * object[key]['physics']
                    print("callback 6, force_local:{}".format(f_rep_local.vec_p))
                f_rep_total += f_rep_local
            # print("f_rep_total:{}".format(f_rep_total.vec_p))
        return f_rep_total

    def activeForce(self, objects, f_active):
        f_delta_load = 0.005
        if self.g_star.length <= 0.005 or self.agent_vel.length >= 0.1:
            pass
        else:
            for key in objects:
                if objects[key]['states'] == 'contacting' and objects[key]['physics'] == 'waiting understanding':
                    agent_to_object = objects[key]['center'] - self.agent_pose
                    f_active = f_active + Vector(agent_to_object.direction)*f_delta_load - self.agent_vel
                else:
                    f_active = Vector([0, 0])
            # print("f_active:{}".format(f_active.vec_p))
        return f_active



if __name__ == "__main__":
    p1 = Vector([2, 5])
    p2 = Vector([5, 1])
    k_att = 1
    workspace_dumping = -2
    object_dumping = -3
    fixed_object_k_rep = 4
    effect = 0.2
    agent_vel = Vector([20, 20])
    object = {'obj_1': {"center":  p1, "attribute": "approaching", "radius": 1}, 'obj_2': {"center": p2, "attribute": "approaching", "radius": 2}}
    goal = Vector([15, 15])
    agent_state = Vector([2, 1])
    workspace = [[0, 0], [20, 20]]
    domain = Domain(effect, agent_state, workspace)
    target_point = domain.TargetPoint(goal)
    print("target_point:{}".format(target_point))
    object_update = domain.ObjectPoint(object)
    apf = Apf(k_att, workspace_dumping, object_dumping, fixed_object_k_rep, effect, agent_state,  agent_vel, workspace)
    att = apf.AttractionForce(target_point)
    rep = apf.RepulFource(object_update)
    print(att.vec_p)
    print(rep.vec_p)