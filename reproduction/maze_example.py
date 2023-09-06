import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from random import choice, randint, seed

from pydrake.geometry.optimization import HPolyhedron
from pydrake.solvers import MosekSolver

from gcs.bezier import BezierGCS
from gcs.linear import LinearGCS
from models.maze import Maze

import util

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

# regions = []
# edges = []
# for x in range(maze_size):
#     for y in range(maze_size):
#         regions.append(HPolyhedron.MakeBox([x, y], [x+1., y+1.]))
#         C = y + x * maze.ny
#         if not maze.map[x][y].walls['N']:
#             edges.append((C, C + 1))
#         if not maze.map[x][y].walls['S']:
#             edges.append((C, C - 1))
#         if not maze.map[x][y].walls['E']:
#             edges.append((C, C + maze.ny))
#         if not maze.map[x][y].walls['W']:
#             edges.append((C, C - maze.ny))
# util.SerializeRegions(regions, './data/maze.csv')
# util.SerializeEdges(edges, './data/maze_edges.csv')

regions = util.DeserializeRegions('./data/maze.csv')
edges = util.DeserializeEdges('./data/maze_edges.csv')


def plot_maze():
    plt.figure(figsize=(5,5))
    plt.axis('equal')
    maze.plot(1)
    plt.plot(*start, 'kx', markersize=10)
    plt.plot(*goal, 'kx', markersize=10)
relaxation = True
gcs = LinearGCS(regions, edges)
# order = 5
# continuity = 3
# gcs = BezierGCS(regions, order, continuity, edges)
gcs.addSourceTarget(start, goal)
gcs.setSolver(MosekSolver())
waypoints, results_dict, best_path = gcs.SolvePath(relaxation)

print("\n")
soln_vid = []
for edge in best_path:
    # print(edge.u().id().get_value(), edge.v().id().get_value(), edge.id().get_value())
    soln_vid.append(edge.u().id().get_value())

soln_vid = [s-1 for s in soln_vid]
# print(soln_vid, sep=", ")


# corres = []
# for i, w in enumerate(waypoints):
#     for j, r in enumerate(regions):
#         print(w)
#         if r.PointInSet(w):
#             corres.append([i, j])

# print(corres)

print(waypoints.shape)
opt_soln = np.loadtxt('./data/opt_soln.txt', delimiter=',')
print(opt_soln.T.shape)

plot_maze()
plt.plot(*waypoints, 'b')
plt.plot(*opt_soln.T, 'r')
plt.show()
