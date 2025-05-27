from os.path import join, dirname, abspath
import sys
import numpy as np
path = join(dirname(dirname(abspath(__file__))))
sys.path.append(path)

from motionPlanner.vector import Vector
import math


def effect_domain(agent_pose, point, effect_para):
    # agent_pose: The position of the agent as a Vector object.
    # point: The target point as a Vector object.
    # effect_para: The effect parameter that scales the direction vector.
    direction = point - agent_pose
    a = [0] * len(direction.vec_p)
    direction_point = Vector(x=a)
    for i in range(len(direction.vec_p)):
        direction_point.vec_p[i] = direction.direction[i] * effect_para
    direction_point.vector_operation()

    return direction_point

def object_domain(agent_pose, point, effect_para):
    # agent_pose: The position of the agent as a Vector object.
    # point: The target point as a Vector object.
    # effect_para: The effect parameter that scales the direction vector.
    direction = point - agent_pose
    a = [0] * len(direction.vec_p)
    direction_point = Vector(x=a)
    for i in range(len(direction.vec_p)):
        direction_point.vec_p[i] = direction.direction[i] * effect_para
    direction_point.vector_operation()
    effect_point = direction_point + agent_pose
    return effect_point


class Domain:
    def __init__(self, effect, workspace, fixed_object_k_rep, object_dumping):
        # effect: The effect parameter defining the influence range.
        # workspace: The workspace boundaries or constraints.
        # fixed_object_k_rep: Repulsion coefficient for fixed objects.
        # object_dumping: Damping coefficient for object interactions.
        self.effect_para = effect
        self.workspace = workspace
        self.fixed_object_k_rep = fixed_object_k_rep
        self.object_dumping = object_dumping
        self.object_effect = 0.08

    def imagined_state(self, agent_state, target_point):
        # agent_state: A dictionary containing the agent's position ('pos') and other state info.
        # target_point: The target position as a Vector or list.
        if isinstance(target_point, list):
            target_point = Vector(target_point)
        direction = target_point - agent_state['pos']
        if direction.length > self.effect_para:
            g_star = effect_domain(agent_state['pos'], target_point, self.effect_para)
        else:
            g_star = target_point - agent_state['pos']
        return g_star

    def object_topology(self, agent_state, object, g_star, target_point):
        # agent_state: A dictionary containing the agent's position ('pos') and state.
        # object: A dictionary of objects, each with 'states', 'attribute', 'center', etc.
        # g_star: The computed direction vector from imaged_state.
        # target_point: The target position as a Vector.
        for key in object:
            if object[key]['states'] == 'approaching' :
                if object[key]['attribute'] == None:
                    direction = object[key]['center'] - agent_state['pos']
                    if direction.length <= 0.143 and g_star.length >= 0.1:
                        # effect = (object[key]["center"]-agent_state['pos']).length - 0.05
                        x_star = object_domain(agent_state['pos'], object[key]["center"], self.effect_para)
                        o_star = object_domain(object[key]["center"], agent_state['pos'], self.effect_para)
                        object[key]['direction'] =  x_star - o_star
                        object[key]['energy_param'] = self.object_dumping
                    elif direction.length <= 0.143 and g_star.length < 0.1:
                        object[key]['direction'] = g_star
                        object[key]['energy_param'] = self.object_dumping
                    else:
                        object[key]['states'] = "away"

                elif object[key]['attribute'] == 'operable':
                    object[key]['direction'] = Vector(g_star.direction)
                    object[key]['energy_param'] = -self.object_dumping/60

                elif object[key]['attribute'] == 'fixed':
                    O_star = effect_domain(object[key]["center"], agent_state['pos'], 0.8)
                    direction = object[key]['center'] - agent_state['pos']
                    if direction.length >= 0.1 and direction.length < 0.142:
                        dis_eval = self.same_side(object[key]['center'].vec_p, Vector([0.262, 0.366]).vec_p, target_point.vec_p, agent_state['pos'].vec_p)
                        coord = self.find_circle_intersections(object[key]["center"].vec_p, agent_state['pos'].vec_p)
                        fixed_param = self.find_acute_vector(coord[0], coord[1], O_star.vec_p,
                                                             object[key]['center'].vec_p, [0.262, 0.366], dis_eval)
                        object[key]['direction'] = Vector(fixed_param)
                        object[key]['energy_param'] = self.fixed_object_k_rep
                    else:
                        object[key]['states'] = "away"

            elif object[key]['states'] == 'contacting':
                if  object[key]['attribute'] == 'operable':
                    object_to_agent = object[key]['center'] - agent_state['pos']
                    proJ = g_star * ((object_to_agent.dot(g_star))/(g_star.length**2))
                    object[key]['direction'] = g_star - proJ
                    object[key]['energy_param'] = object[key]['physics']

                elif object[key]['attribute'] == 'fixed':
                    O_star = object_domain(object[key]["center"], agent_state['pos'], 0.08)
                    direction = object[key]['center'] - agent_state['pos']
                    if direction.length < 0.143:
                        dis_eval = self.same_side(object[key]['center'].vec_p, Vector([0.262, 0.366]).vec_p, target_point.vec_p, agent_state['pos'].vec_p)
                        coord = self.find_circle_intersections(object[key]["center"].vec_p, agent_state['pos'].vec_p)
                        fixed_param = self.find_acute_vector(coord[0], coord[1], O_star.vec_p, object[key]['center'].vec_p,[0.262, 0.366], dis_eval)
                        object[key]['direction'] = Vector(fixed_param.tolist())
                        object[key]['energy_param'] = self.fixed_object_k_rep
                    else:
                        object[key]['states'] = "away"
        return object

    def same_side(self, x1, x2, x3, x4):
        """
        Check if points x3 and x4 are on the same side of the line defined by x1 and x2.
        """
        (x1_val, y1_val) = x1
        (x2_val, y2_val) = x2
        (x3_val, y3_val) = x3
        (x4_val, y4_val) = x4

        # Calculate the cross products
        cross_product3 = (x2_val - x1_val) * (y3_val - y1_val) - (y2_val - y1_val) * (x3_val - x1_val)
        cross_product4 = (x2_val - x1_val) * (y4_val - y1_val) - (y2_val - y1_val) * (x4_val - x1_val)

        if (cross_product3 * cross_product4) > 0:
            return 1
        else:
            return 0

    def find_point(self, d1, d2, option, agent_pos):
        # d1: First candidate direction vector.
        # d2: Second candidate direction vector.
        # option: A flag (0 or 1) indicating selection logic.
        # agent_pos: The agent's current position.
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

    def find_acute_vector(self, x1, x2, x3, x4, x5, dis_eval):
        """
        Find which vector (x3-x1 or x3-x2) forms an acute angle with (x5-x4).

        Parameters:
        points : list or array-like
            A sequence of five points in format [x1, x2, x3, x4, x5],
            where each point can be any dimensional (but x1-x5 must have same dimension).

        Returns:
        numpy.ndarray
            The vector (either x3-x1 or x3-x2) that forms an acute angle with (x5-x4).
            Returns None if neither forms an acute angle (shouldn't happen unless vectors are orthogonal).

        Notes:
        - An acute angle is determined by positive dot product (angle < 90 degrees).
        - If both vectors form acute angles, returns the first one (x3-x1).
        """

        # Construct the candidate vectors
        vec_a = np.array(x3) - np.array(x1)  # First candidate vector
        vec_b = np.array(x3) - np.array(x2)  # Second candidate vector
        vec_target = np.array(x5) - np.array(x4)  # The reference vector

        # Calculate dot products
        dot_a = np.dot(vec_a, vec_target)
        dot_b = np.dot(vec_b, vec_target)

        # Determine which vector forms an acute angle
        if dis_eval == 0:
            return vec_a if dot_a > 0 else vec_b
        else:
            return np.array([0, 0])

    def find_circle_intersections(self, x1, x2):
        """
        Find the intersection points of two circles.

        Parameters:
        x1, y1 (float): Coordinates of the center of the first circle.
        r1 (float): Radius of the first circle.
        x2, x2[1] (float): Coordinates of the center of the second circle.
        r2 (float): Radius of the second circle.

        Returns:
        list: A list of intersection points as tuples [(x3, y3), (x4, y4)], or an empty list if no intersections exist.
        """

        # Calculate the distance between the centers of the two circles
        dx = x2[0] - x1[0]
        dy = x2[1] - x1[1]
        d = np.sqrt(dx ** 2 + dy ** 2)

        # Check if circles are separate, concentric, or one is contained within the other without touching
        if d > self.object_effect + self.effect_para or d < abs(self.object_effect - self.effect_para):
            return []  # No intersections

        # Check if circles are coincident (infinite intersections)
        if d == 0 and self.object_effect == self.effect_para:
            raise ValueError("The circles are coincident and have infinite intersections.")

        # Calculate the distance from circle1's center to the line joining the intersection points
        a = (self.object_effect ** 2 - self.effect_para ** 2 + d ** 2) / (2 * d)

        # Calculate the height of the intersection points from the line joining the centers
        h = np.sqrt(self.object_effect ** 2 - a ** 2)

        # Calculate the coordinates of the midpoint P on the line joining the centers
        Px = x1[0] + (a * dx) / d
        Py = x1[1] + (a * dy) / d

        # Calculate the two intersection points
        # Offset from P in the direction perpendicular to the line joining the centers
        x3 = Px + (h * dy) / d
        y3 = Py - (h * dx) / d
        x4 = Px - (h * dy) / d
        y4 = Py + (h * dx) / d

        # If the circles are tangent, return the single point
        if np.isclose(x3, x4) and np.isclose(y3, y4):
            return [(x3, y3)]

        return [(x3, y3), (x4, y4)]




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









