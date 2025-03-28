import math
import numpy as np

class Vector:

    def __init__(self, x, y=None, z=None):
        if isinstance(x, list):
            self.vec_p = [0] * len(x)
            for i in range(len(x)):
                self.vec_p[i] = x[i]
        elif isinstance(x, int) or isinstance(x, float):
            self.vec_p = [0] * 2
            self.vec_p[0] = x
            self.vec_p[1] = y
            if z != None:
                self.vec_p = [0] * 3
                self.vec_p[0] = x
                self.vec_p[1] = y
                self.vec_p[2] = z
        elif isinstance(x, np.ndarray):
            self.vec_p = x
        self.vector_operation()

    def vector_operation(self):
        l = 0
        self.direction = [0]*len(self.vec_p)
        for i in range(len(self.vec_p)):
            l += self.vec_p[i]**2
        self.length = math.sqrt(l)
        if self.length == 0:
            self.direction = [0]*len(self.vec_p)
        else:
            for i in range(len(self.vec_p)):
                self.direction[i] = self.vec_p[i] / self.length

    def __add__(self, other):
        a = [0]*len(self.vec_p)
        r = Vector(a)
        for i in range(len(self.vec_p)):
            r.vec_p[i] = self.vec_p[i] + other.vec_p[i]
        r.vector_operation()
        return r

    def __sub__(self, other):
        a = [0] * len(self.vec_p)
        r = Vector(x=a)
        for i in range(len(self.vec_p)):
            r.vec_p[i] = self.vec_p[i] - other.vec_p[i]
        r.vector_operation()
        return r

    def __mul__(self, other):
        a = [0] * len(self.vec_p)
        r = Vector(x=a)
        for i in range(len(r.vec_p)):
            r.vec_p[i] = self.vec_p[i] * other
        r.vector_operation()
        return r

    def __truediv__(self, other):
        a = [0] * len(self.vec_p)
        r = Vector(a)
        for i in range(len(r.vec_p)):
            r.vec_p[i] = self.vec_p[i] / other
        r.vector_operation()
        return r

    def dot(self, other):
        scalar_product = 0
        for i in range((len(self.vec_p))):
            scalar_product += self.vec_p[i] * other.vec_p[i]
        return scalar_product


if __name__=="__main__":
    v = [2, 3, 4]
    v0 = [3, 7, 8]

    v1 = Vector(v)
    v2 = Vector(v0)
    v8 = Vector(v2.vec_p)

    v3 = v1.dot(v2)
    v4 = v1 / 2
    # print("v1.vec_p:{}; v2.vec_p:{}; v1.length:{}; v1.direction:{}".format(v1.vec_p, v2.vec_p, v1.length, v1.direction))
    print(f'v3:{v3}')