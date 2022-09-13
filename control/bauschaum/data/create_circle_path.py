#!/usr/bin/env python3

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import CirclePolygon
import numpy as np

def write_path_funcs(path, file_name='path', func_name=None, axisName=["x","y","z"]):
    if func_name is None:
        func_name = file_name.lower()
    for i in range(path.shape[1]):
        write_points(path[:,i], file_name+axisName[i]+'.py', func_name+axisName[i])

def write_points(points, file_name='path.py', func_name=None):
    if func_name is None:
        func_name = file_name.lower()[:-3]
    with open (file_name, 'w') as f:
        f.write('def '+func_name+'():\n')
        f.write('\treturn [')
        for i in range(len(points)):
            f.write(str(points[i]))
            if i != len(points)-1:
                f.write(',')
            else:
                f.write(']')


def get_path_points(circle):
    verts = circle.get_path().vertices
    trans = circle.get_patch_transform()
    points = trans.transform(verts)
    return points

circle_mir = CirclePolygon((0, 0), radius = 5.0, fc = 'y', resolution=4000)
circle_ur = CirclePolygon((0, 0), radius = 4.5, fc = 'y', resolution=4000)



p_mir = get_path_points(circle_mir)
p_ur = get_path_points(circle_ur)

# p_mir=np.flip(p_mir, axis=0)
# p_ur=np.flip(p_ur, axis=0)

p_mir=p_mir[:-2000:-1,:]
p_ur=p_ur[-21:-2020:-1,:]

p_ur = np.append(p_ur,np.ones((p_ur.shape[0],1))*0.35, axis=1)

layers=2
layer_h = 0.3
p_ur_base = p_ur.copy()
p_mir_base = p_mir.copy()
for i in range(1,layers):
    p_ur_layer = p_ur_base.copy()
    p_ur_layer[:,2] = p_ur_layer[:,2] + i*layer_h
    p_ur = np.append(p_ur,p_ur_layer, axis=0)
    p_mir = np.append(p_mir,p_mir_base, axis=0)

fig = plt.figure()
ax = Axes3D(fig)



ax.plot(p_ur[:,0], p_ur[:,1], p_ur[:,2])
# ax.plot(p_mir[:,0], p_mir[:,1], p_mir[:,2])
# plt.plot(p_mir[100:2000,0], p_mir[100:2000,1])
# plt.plot(p_ur[2000:,0], p_ur[2000:,1])
plt.show()

write_path_funcs(p_mir, 'circle_mir')
write_path_funcs(p_ur, 'circle_ur')