import os
import numpy as np
import matplotlib.pyplot as plt
from random import choice, randint, seed

from pydrake.solvers import MosekSolver
from pydrake.planning import GcsTrajectoryOptimization
from pydrake.geometry.optimization import ConvexSet
from scipy.optimize import linprog

from gcs.linear import LinearGCS
from models.maze import Maze

from gcs import util

os.environ["MOSEKLM_LICENSE_FILE"] = "/home/gaussian/Documents/softwares/mosektoolslinux64x86/mosek.lic"
MosekSolver.AcquireLicense()
print("Mosek is enabled: ", MosekSolver().enabled())

seed(4)

maze_size = 50
knock_downs = 100

start = np.array([0.5, 0])
goal = np.array([maze_size - 0.5, maze_size])
maze = Maze(maze_size, maze_size)
maze.make_maze()

while knock_downs > 0:
    cell = maze.cell_at(randint(1, maze_size - 2), randint(1, maze_size - 2))
    walls = [w for w, up in cell.walls.items() if up]
    if len(walls) > 0:
        maze.knock_down_wall(cell, choice(walls))
        knock_downs -= 1

regions = util.DeserializeRegions('../data/maze.csv')
edges = util.DeserializeEdges('../data/maze_edges.csv')

def is_monotonically_increasing(arr):
    return np.all(arr[1:] >= arr[:-1])

def plot_maze():
    plt.figure(figsize=(5,5))
    plt.axis('equal')
    maze.plot(1)
    plt.plot(*start, 'kx', markersize=10)
    plt.plot(*goal, 'kx', markersize=10)

num_pos = np.size(start)
order = 1
h_min = 1e-2
h_max = 1.0

# full_path = [1, 51, 52, 2, 3, 4, 54, 104, 154, 204, 203, 202, 252, 302, 301, 351, 401, 451, 501, 551, 552, 553, 554, 504, 505, 506, 556, 557, 507, 457, 407, 406, 456, 455, 405, 355, 356, 306, 305, 304, 303, 253, 254, 255, 205, 206, 256, 257, 207, 157, 156, 155, 105, 106, 56, 57, 7, 8, 9, 59, 58, 108, 109, 110, 111, 112, 62, 61, 11, 12, 13, 63, 64, 14, 15, 16, 66, 116, 115, 114, 164, 163, 162, 161, 211, 210, 260, 259, 209, 208, 258, 308, 358, 408, 409, 410, 411, 461, 462, 512, 513, 463, 464, 414, 415, 416, 466, 516, 517, 518, 468, 467, 417, 418, 419, 369, 319, 320, 270, 271, 321, 322, 372, 422, 472, 471, 470, 520, 570, 571, 572, 622, 623, 624, 574, 524, 525, 575, 576, 626, 676, 726, 776, 777, 827, 877, 878, 828, 829, 830, 831, 881, 882, 932, 982, 1032, 1033, 1083, 1133, 1134, 1135, 1185, 1184, 1183, 1233, 1283, 1333, 1332, 1382, 1383, 1433, 1483, 1533, 1583, 1584, 1534, 1535, 1485, 1435, 1385, 1386, 1436, 1486, 1487, 1537, 1587, 1586, 1585, 1635, 1685, 1735, 1785, 1835, 1885, 1935, 1936, 1937, 1987, 2037, 2087, 2086, 2136, 2186, 2185, 2235, 2285, 2286, 2287, 2237, 2238, 2239, 2289, 2290, 2340, 2341, 2342, 2343, 2344, 2294, 2244, 2245, 2195, 2196, 2197, 2247, 2246, 2296, 2346, 2396, 2446, 2496, 2497, 2447, 2448, 2449, 2499, 2500]
# full_path = [1, 51, 52, 2, 3, 4, 54, 104, 154, 204, 203, 202, 252, 302, 301, 351, 401, 451, 501, 551, 552, 553, 554, 504, 505, 506, 556, 557, 507, 457, 407, 406, 456, 455, 405, 355, 356]
full_path = [1, 51, 52, 2, 3, 4, 54, 104, 154, 204, 203, 202, 252, 302, 301, 351, 401, 451, 501, 551, 552, 553, 554, 504, 505, 506, 556, 557, 507, 457, 407, 406, 456, 455, 405, 355, 356, 306, 305, 304, 303, 253, 254, 255, 205, 206, 256, 257, 207, 157, 156, 155, 105, 106, 56, 57, 7, 8, 9, 59, 58, 108, 109, 110, 111, 112, 62, 61, 11, 12, 13, 63, 64, 14, 15, 16, 66, 116, 115, 114, 164, 163, 162, 161, 211, 210, 260, 259, 209, 208, 258, 308, 358, 408, 409, 410, 411, 461, 462, 512, 513, 463, 464, 414, 415, 416, 466, 516, 517, 518, 468, 467, 417, 418, 419, 369]

region_centers = []
for cell in full_path:
    r = regions[cell-1]
    ctrs = r.ChebyshevCenter()
    region_centers.append(ctrs)

opt_type = 'gcsopt'
t1 = np.loadtxt('../data/monotonicity/' + opt_type + '/t1.txt')
t2 = np.loadtxt('../data/monotonicity/' + opt_type + '/t2.txt')
t3 = np.loadtxt('../data/monotonicity/' + opt_type + '/t3.txt')
t4 = np.loadtxt('../data/monotonicity/' + opt_type + '/t4.txt')
t5 = np.loadtxt('../data/monotonicity/' + opt_type + '/t5.txt')

plot_maze()
for i, ctrs in enumerate(region_centers):
    print("Center at cell:", full_path[i], "is", ctrs)
    plt.scatter(ctrs[0], ctrs[1], s=200, marker='s', alpha=0.3, c='red', edgecolors='green')
    plt.text(ctrs[0], ctrs[1], full_path[i], fontsize=8, ha='center', va='center')
plt.plot(*t1.T, label='t1')
plt.plot(*t2.T, label='t2')
plt.plot(*t3.T, label='t3')
# plt.plot(*t4.T, label='t4')
# plt.plot(*t5.T, label='t5')

plt.xlim([0, 9])
plt.ylim([0, 20])
plt.legend(loc='upper left')
plt.savefig('../data/cmp_opt_monot.pdf')
plt.show()

