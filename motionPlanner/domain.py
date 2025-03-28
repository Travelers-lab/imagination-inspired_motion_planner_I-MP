from os.path import join, dirname, abspath
import sys

path = join(dirname(dirname(abspath(__file__))))
sys.path.append(path)

from motionPlanner.vector import Vector
import math


def effect_domain(agent_pose, point, effect_para):
    direction = point - agent_pose
    a = [0]*len(direction.vec_p)
    effect_point = Vector(x=a)
    for i in range(len(direction.vec_p)):
        effect_point.vec_p[i] = direction.direction[i] * effect_para
    effect_point.vector_operation()
    return effect_point


class Domain:
    def __init__(self, effect, workspace, fixed_object_k_rep, object_dumping):
        self.effect_para = effect
        self.workspace = workspace
        self.fixed_object_k_rep = fixed_object_k_rep
        self.object_dumping = object_dumping

    def imaged_state(self, agent_state, target_point):
        if isinstance(target_point, list):
            target_point = Vector(target_point)
        direction = target_point - agent_state['pos']
        if direction.length > self.effect_para:
            g_star = effect_domain(agent_state['pos'], target_point, self.effect_para)
        else:
            g_star = target_point - agent_state['pos']
        return g_star

    def object_topology(self, agent_state, object, g_star, target_point):
        for key in object:
            if object[key]['states'] == 'approaching' :
                if object[key]['attribute'] == None:
                    direction = object[key]['center'] - target_point
                    if direction.length < 0.142 and g_star.length >= 0.1:
                        effect = (object[key]["center"]-agent_state['pos']).length - 0.05
                        x_star = effect_domain(agent_state['pos'], object[key]["center"], self.effect_para)
                        o_star = effect_domain(agent_state['pos'], object[key]["center"], effect)
                        object[key]['direction'] =  o_star - x_star
                        object[key]['energy_param'] = [self.object_dumping]
                    elif direction.length < 0.142 and g_star.length < 0.1:
                        object[key]['direction'] = g_star
                        object[key]['energy_param'] = [self.object_dumping]
                    else:
                        object[key]['direction'] = Vector([0, 0])
                        object[key]['energy_param'] = [0]

                elif object[key]['attribute'] == 'operable':
                    object[key]['direction'] = g_star
                    object[key]['energy_param'] = object[key]['physics']

                elif object[key]['attribute'] == 'fixed':
                    O_star = effect_domain(object[key]["center"], agent_state['pos'], object[key]['radius'])
                    direction = object[key]['center'] - agent_state['pos']
                    if direction.length >= 0.1 and direction.length < 0.142:
                        t = math.atan2(direction.vec_p[0], direction.vec_p[1])
                        d = (direction.length ** 2 + self.effect_para ** 2 - object[key]['radius'] ** 2) / (
                                    2 * direction.length * self.effect_para)
                        G_max1 = Vector([agent_state['pos'].vec_p[0] + self.effect_para * math.cos(t + d),
                                  agent_state['pos'].vec_p[1] + self.effect_para * math.sin(t + d)])
                        G_max2 = Vector([agent_state['pos'].vec_p[0] + self.effect_para * math.cos(t - d),
                                  agent_state['pos'].vec_p[1] + self.effect_para * math.sin(t - d)])
                        option = self.same_side(target_point, object[key]['center'], Vector([0.262, 0.366]), agent_state['pos'])
                        d1 = G_max1 - target_point
                        d2 = G_max2 - target_point
                        rep_center = self.find_point(d1, d2, option, agent_state['pos'])
                        object[key]['direction'] = O_star - rep_center
                        object[key]['energy_param'] = [self.fixed_object_k_rep]
                    else:
                        object[key]['direction'] = Vector([0, 0])
                        object[key]['energy_param'] = [self.fixed_object_k_rep]

            elif object[key]['states'] == 'contacting':
                if  object[key]['attribute'] == 'operable':
                    direction = object[key]['center'] - target_point
                    if direction.length < 0.142 and g_star.length >= 0.1:
                        o_star = effect_domain(agent_state['pos'], object[key]["center"], self.effect_para)
                        g_star = effect_domain(agent_state['pos'], target_point, self.effect_para)
                        object[key]['direction'] = g_star*(o_star.dot(g_star)/g_star.dot(g_star))
                        object[key]['energy_param'] = object[key]['physics']
                    elif direction.length < 0.142 and g_star.length < 0.1:
                        object[key]['direction'] = Vector(g_star.direction)
                        object[key]['energy_param'] = [0.05*self.object_dumping]
                    else:
                        object[key]['direction'] = Vector([0, 0])
                        object[key]['energy_param'] = [0]

                elif object[key]['attribute'] == 'fixed':

                    O_star = effect_domain(object[key]["center"], agent_state['pos'], object[key]['radius'])
                    direction = object[key]['center'] - agent_state['pos']
                    if direction.length >= 0.1 and direction.length < 0.142:
                        t = math.atan2(direction.vec_p[0], direction.vec_p[1])
                        d = (direction.length ** 2 + self.effect_para ** 2 - object[key]['radius'] ** 2) / (
                                    2 * direction.length * self.effect_para)
                        G_max1 = Vector([agent_state['pos'].vec_p[0] + self.effect_para * math.cos(t + d),
                                  agent_state['pos'].vec_p[1] + self.effect_para * math.sin(t + d)])
                        G_max2 = Vector([agent_state['pos'].vec_p[0] + self.effect_para * math.cos(t - d),
                                  agent_state['pos'].vec_p[1] + self.effect_para * math.sin(t - d)])
                        option = self.same_side(target_point, object[key]['center'], Vector([0.262, 0.366]), agent_state['pos'])
                        d1 = G_max1 - target_point
                        d2 = G_max2 - target_point
                        rep_center = self.find_point(d1, d2, option, agent_state['pos'])
                        object[key]['direction'] = O_star - rep_center
                        object[key]['energy_param'] = [self.fixed_object_k_rep]
                    else:
                        object[key]['direction'] = Vector([0, 0])
                        object[key]['energy_param'] = [self.fixed_object_k_rep]

        # print("object[key]['direction']:{}".format(object[key]['direction'].vec_p))
        return object

    def same_side(self, target_point, object_center, robot_center, agent_pos):
        def cross_product(a, b, c):
            return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        cp3 = cross_product(target_point.vec_p, object_center.vec_p, robot_center.vec_p)
        cp4 = cross_product(target_point.vec_p, object_center.vec_p, agent_pos.vec_p)
        if (cp3 * cp4) > 0:
            return 1
        else:
            return 0

    def find_point(self, d1, d2, option, agent_pos):
        if option == 1:
            return d1 if d1.length > d2.length else d2
        else:
            if d1.length > d2.length:
                return d2
            elif d1.length == d2.length:
                dis_eval = self.same_side(target_point, object[key]['center'], Vector([0.262, 0.366]), agent_pos)
                return d1 if dis_eval == 1 else d2
            else:
                return d2




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









