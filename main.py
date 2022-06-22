from ctypes import pointer
import numpy as np
import matplotlib.pyplot as plt

H_FOV = 54.4 * np.pi / 180
O_R = 0.25
    

def draw_camera_fov_from_position(position, th, h_fov, depth=1, color='b'):
    v1 = position
    l_size = np.sin(h_fov/2)
    v2 = position + depth * (l_size * np.array([-np.sin(th), np.cos(th)]) + np.array([np.cos(th), np.sin(th)]))
    v3 = position - depth * (l_size * np.array([-np.sin(th), np.cos(th)]) - np.array([np.cos(th), np.sin(th)]))
    return plt.Polygon(np.array([v1,v2,v3]), color=color)

def draw_camera_from_position(position, th, h_fov):
    return draw_camera_fov_from_position(position, th, h_fov, 0.5)

def compute_occlusion(c, c_th, c_fov, o):
    co = o - c
    co /= np.linalg.norm(co)
    perp_co = np.array([-co[1], co[0]])
    o1 = o + perp_co * O_R
    o2 = o - perp_co * O_R
    c_o1 = o1 - c
    c_o1 /= np.linalg.norm(c_o1)
    c_o2 = o2 - c
    c_o2 /= np.linalg.norm(c_o2)
    o1_f = o1 + c_o1 * 10
    o2_f = o2 + c_o2 * 10
    return np.array([o1,o2,o1_f,o2_f])

fig, ax = plt.subplots()
ax.set_aspect('equal', adjustable='datalim')
c_pos = np.array([2,1])
c_th = 0
o_pos = np.array([1,2])

o_circ = plt.Circle(o_pos, O_R, color='r')
c_shape = draw_camera_from_position(c_pos, c_th, H_FOV)
c_fov_shape = draw_camera_fov_from_position(c_pos, c_th, H_FOV, 10, color='y')
occlusion_shape = plt.Polygon(compute_occlusion(c_pos, c_th, H_FOV, o_pos), color='o')

ax.add_patch(c_fov_shape)
ax.add_patch(o_circ)
ax.add_patch(c_shape)
ax.add_patch(occlusion_shape)

ax.set_ylim([-5, 5])
ax.set_xlim([-5, 5])
plt.show()