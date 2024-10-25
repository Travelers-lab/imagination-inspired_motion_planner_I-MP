from os.path import join, dirname, abspath
import sys
path = join(dirname(dirname(abspath(__file__))))
sys.path.append(path)

from lunch.new_domain.Vector import Vector
import math


def EffectDomain(agent_pose, point, effect_para):
    if isinstance(agent_pose, list):
        agent_pose = Vector(agent_pose)
    if isinstance(point, list):
        point = Vector(point)
    direction = point - agent_pose
    a = [0]*len(direction.vec_p)
    effect_point = Vector(x=a)
    for i in range(len(direction.vec_p)):
        effect_point.vec_p[i] = direction.direction[i] * effect_para
    effect_point.vector_operation()
    #print("format:{}".format(effect_point.direction))
    return effect_point


class Domain:
    def __init__(self, effect, agent_pose, workspace):
        if isinstance(agent_pose, list):
            agent_pose = Vector(agent_pose)
        self.effect_para = effect
        self.agent_pose = agent_pose
        self.workspace = workspace

    def TargetPoint(self, target_point):
        if isinstance(target_point, list):
            target_point = Vector(target_point)
        direction = target_point - self.agent_pose
        # print("direction to goal:{}".format(direction.length))
        if direction.length > self.effect_para:
            attraction_point = EffectDomain(self.agent_pose, target_point, self.effect_para)
        else:
            attraction_point = target_point - self.agent_pose
        return attraction_point

    def ObjectPoint(self, object):
        for key in object:
            if object[key]['states'] == 'approaching' and object[key]['attribute'] == None:
                O_star = EffectDomain(object[key]["center"], self.agent_pose, object[key]['radius'])
                X_star = EffectDomain(self.agent_pose, object[key]["center"], self.effect_para)
                object[key]['direction'] = O_star - X_star
            elif object[key]['states'] == 'approaching' and object[key]['attribute'] == 'fixed':
                O_star = EffectDomain(object[key]["center"], self.agent_pose, object[key]['radius'])
                direction = object[key]['center'] - self.agent_pose
                print("direction.length:{}".format(direction.length))
                if direction.length >= 0.1 and direction.length < 0.15:
                    t = math.atan2(direction.vec_p[0], direction.vec_p[1])
                    d = (direction.length**2 + self.effect_para**2 - object[key]['radius']**2)/(2*direction.length*self.effect_para)
                    G_max1 = [self.agent_pose.vec_p[0]+self.effect_para*math.cos(t+d), self.agent_pose.vec_p[1]+self.effect_para*math.sin(t+d)]
                    G_max2 = [self.agent_pose.vec_p[0]+self.effect_para*math.cos(t-d), self.agent_pose.vec_p[1]+self.effect_para*math.sin(t-d)]
                    workspace_center = [(self.workspace[0][0]+self.workspace[1][0])/2, (self.workspace[0][1]+self.workspace[1][1])/2]
                    s_abc = (self.agent_pose.vec_p[0]-workspace_center[0])*(object[key]['center'].vec_p[1]-workspace_center[1]) - (self.agent_pose.vec_p[1]-workspace_center[1])*(object[key]['center'].vec_p[0]-workspace_center[0])
                    if s_abc > 0:
                        max_repel = Vector(G_max1) - object[key]["center"]
                    else:
                        max_repel = Vector(G_max2) - object[key]["center"]
                    object[key]['direction'] = O_star - max_repel
                else:
                    object[key]['direction'] = Vector([0]*len(self.agent_pose.vec_p))
        # print("object[key]['direction']:{}".format(object[key]['direction'].vec_p))
        return object




if __name__ == "__main__":
    p1 = Vector([2, 5])
    p2 = Vector([5, 1])
    object = {'obj_1': {"center":  p1, "attribute": "approaching", "radius": 1}, 'obj_2': {"center": p2, "attribute": "fixed", "radius": 2}}
    target_point = Vector([15, 15])
    agent_state = Vector([2, 1])
    workspace = [[0, 0], [20, 20]]
    domain = Domain(3, agent_state, workspace)
    target_point = domain.TargetPoint(target_point)
    object_update = domain.ObjectPoint(object)
    print("target:{}".format(target_point.vec_p))
    print(object_update)
    for key in object_update:
        print("total directio:{}".format(object_update[key]['direction'].vec_p))









