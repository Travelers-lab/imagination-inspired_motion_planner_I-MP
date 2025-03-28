import numpy as np
from motionPlanner.vector import Vector

b = np.array((0.5, 0.5)) * np.array((0.5, 0.5)) + np.array((0.5, 0.5)) * np.array((0.5, 0.5))

A = [[0.44, 0.37], [0.6, 0.1], [0.75, 0.35], [0.6, 0.55]]
road = 0
for i in range(len(A)-1):
    C = Vector(A[i])
    D = Vector(A[i+1])
    length = (D-C).length
    road += length
    print(road)
