from os.path import join, dirname, abspath
import sys
path = join(dirname(dirname(abspath(__file__))), "stateEstimation")
sys.path.append(path)
print(path)
import math
import numpy as np

import matplotlib.pyplot as plt
import pybullet as p
import time
import pybullet_data

direct = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
# jiazai huanjing wuti
p.loadURDF("plane.urdf")
p.loadURDF("r2d2.urdf", [0, 0, 1])
cube_trans = p.loadURDF("cube_small.urdf", basePosition=[0.0, 0.1, 0.025])

p.changeVisualShape(cube_trans, -1, rgbaColor=[1, 1, 1, 0.1])

# xiang ji can shu
width = 128
heigth = 128

fov = 60
aspect = width / heigth
near = 0.02
far = 1

view_matrix = p.computeViewMatrix([0, 0, 5], [0, 0, 0], [1, 0, 0])
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
while 1:
    images = p.getCameraImage(
    width,
    heigth,
    view_matrix,
    projection_matrix,
    shadow=True,
    renderer=p.ER_BULLET_HARDWARE_OPENGL
    )

    rgb_opengl = np.reshape(images[2], (width, heigth, 4)) * 1. / 255
    depth_buffer_opengl = np.reshape(images[3], [width, heigth])
    depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
    seg_opengl = np.reshape(images[4], [width, heigth]) * 1. / 255
    plt.subplot(3, 1, 1)
    plt.imshow(depth_opengl, cmap="gray", vmin=0, vmax=1)
    plt.title("depth OpenGl3")

    plt.subplot(3, 1, 2)
    plt.imshow(rgb_opengl)
    plt.title("RGB OpenGl3")

    plt.subplot(3, 1, 3)
    plt.imshow(seg_opengl)
    plt.title("RGB OpenGl3")
    p.stepSimulation()
    time.sleep(1)







# a = np.array([[1,1,1,1],[2,2,2,2],[3,3,3,3],[4,4,4,4]])
# b = [1,2,3,4]
# c = a.dot(b)[:3]
# d = b[::-1]
# print(d)
# a = [2.0944, 3.14159, 4.18879, 5.235987]
# b = 0.5239
#
# c = []
# for i in range(len(a)):
#     c.append(a[i] + b)
#
# print(c)

# a = [0, 45, 90, 135, 180, 225, 270, 315]
# a = ((np.array(a)+22.5)/180)*3.14
# b1 = []
# b2 = []
# for i in a:
#     b1.append(math.sin(i))
#     b2.append(math.cos(i))
# print(a)
# c = 0.044
# c_y = []
# c_z = []
# for i in b1:
#     c_z.append(c*i)
#
# for i in b2:
#     c_y.append(c*i)
#
# print("y:{}\nz:{}".format(c_y, c_z))


# a = [-10, 40, 90, 140]
# a = ((np.array(a)+22.5)/180)*3.14
# b1 = []
# b2 = []
# for i in a:
#     b1.append(math.sin(i))
#     b2.append(math.cos(i))
# print(a)
# c = -0.051
# c_y = []
# c_x = []
# for i in b1:
#     c_x.append(c*i)
#
# for i in b2:
#     c_y.append(c*i)
#
# print("x:{}\n y:{}".format(c_x, c_y))


# a = [-110, -55, 0, 55, 110]
# a = ((np.array(a))/180)*3.14159
# b1 = []
# b2 = []
# for i in a:
#     b1.append(math.sin(i))
#     b2.append(math.cos(i))
# print(a)
# c = 0.04357
# c_y = []
# c_z = []
# for i in b1:
#     c_z.append(c*i)
#
# for i in b2:
#     c_y.append(c*i)
#
# print("x:{}\ny:{}".format(c_y, c_z))


