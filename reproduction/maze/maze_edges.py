import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from random import choice, randint, seed

# from pydrake.geometry.optimization import HPolyhedron
from pydrake.solvers import MosekSolver

from gcs.bezier import BezierGCS
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

def plot_maze():
    plt.figure(figsize=(5,5))
    plt.axis('equal')
    maze.plot(1)
    plt.plot(*start, 'kx', markersize=10)
    plt.plot(*goal, 'kx', markersize=10)

temp = 0
edge_lines = []
annot_edges = []
for e in edges:
    temp += 1
    # if temp > 7:
    #     break
    if e[0] < 2490 or e[1] < 2490:
        continue
    c1 = regions[e[0]].ChebyshevCenter()
    c2 = regions[e[1]].ChebyshevCenter()
    edge_lines.append([c1, c2])
    annot_edges.append(e)

plot_maze()
for l, e in zip(edge_lines, annot_edges):
    plt.plot([l[0][0], l[1][0]], [l[0][1], l[1][1]], 'b-')
    plt.text(l[0][0], l[0][1], str(e[0]))
    plt.text(l[1][0], l[1][1], str(e[1]))


# plt.xlim([0,2])
# plt.ylim([0,10])
plt.xlim([48,50])
plt.ylim([40,50])
plt.show()
