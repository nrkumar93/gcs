import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
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
lbg_cost = np.loadtxt('../data/maze_lbg.txt', delimiter=' ')

xx = []
yy = []
col = []
for val in lbg_cost:
    vtx = val[0]
    if vtx > len(regions):
        continue
    ctr = regions[int(vtx)-1].ChebyshevCenter()
    xx.append(ctr[0])
    yy.append(ctr[1])
    col.append(val[1])

def plot_maze():
    plt.figure(figsize=(5,5))
    plt.axis('equal')
    maze.plot(1)
    plt.plot(*start, 'kx', markersize=10)
    plt.plot(*goal, 'kx', markersize=10)

plot_maze()
# Create a scatter plot with square markers and colors
# plt.scatter(xx, yy, s=10, c=col, cmap='viridis', norm=mpl.colors.LogNorm())
plt.scatter(xx, yy, s=10, c=col, cmap='viridis')
# Add a color bar for reference
cbar = plt.colorbar()
cbar.set_label('LBG cost')
plt.savefig('../data/maze_lbg_cost.pdf')
plt.show()
