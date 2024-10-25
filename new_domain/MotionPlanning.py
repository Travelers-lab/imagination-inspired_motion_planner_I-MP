from os.path import join, dirname, abspath
import sys
path = join(dirname(dirname(abspath(__file__))))
sys.path.append(path)

from lunch.new_domain.Apf import Apf
from lunch.new_domain.Domain import Domain
from lunch.new_domain.Vector import Vector
import math

class MotionPlanning:

    def __init__(self, k_att, workspace_dumping, object_dumping, fixed_object_k_rep, effect, agent_pose,  agent_vel, workspace,target_point, object):
        self.target_point = target_point
        self.object = object
        self.agent_pose = agent_pose
        self.apf = Apf(k_att, workspace_dumping, object_dumping, fixed_object_k_rep, agent_vel)
        self.domain = Domain(effect, agent_pose, workspace)
        self.f_active = [0, 0]

    def TotalForce(self):
        g_star = self.domain(self.target_point)
        object_update = self.domain(self.object)
        f_att = self.apf.AttractionForce(g_star)
        f_rep = self.apf.RepulFource(object_update)
        f_total = f_att + f_rep
        return f_total


def motionPlanning(effect, agent_pose, agent_vel, workspace, target_point, objects, f_active, hsSensorInfo):
    facter = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1, 2, 3, 5, 10, 50]
    k_att = facter[-2]
    # workspace_dumping = -2*math.sqrt(k_att)
    workspace_dumping = 0
    object_dumping = -2
    fixed_object_k_rep = 0.2
    domain = Domain(effect, agent_pose, workspace)
    target = domain.TargetPoint(target_point)
    object_update = domain.ObjectPoint(objects)
    apf = Apf(k_att=k_att, workspace_dumping=workspace_dumping, object_dumping=object_dumping, fixed_object_k_rep=fixed_object_k_rep,
              effect=effect, agent_pose=agent_pose, agent_vel=agent_vel, workspace=workspace, g_star=target)
    f_att = apf.AttractionForce()
    f_rep = apf.RepulFource(object_update)
    f_active = apf.activeForce(objects=objects, f_active=f_active)
    force = f_att + f_rep + f_active
    # print("force_total:{}".format(force.vec_p))
    f_tot = [0]*3
    for i in range(len(force.vec_p)):
        f_tot[i] = force.vec_p[i]
    return f_tot, f_att, f_rep, f_active



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