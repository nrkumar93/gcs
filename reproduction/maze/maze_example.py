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

regions = util.DeserializeRegions('../data/maze2d/maze.csv')
edges = util.DeserializeEdges('../data/maze2d/maze_edges.csv')


def plot_maze():
    plt.figure(figsize=(5,5))
    plt.axis('equal')
    maze.plot(1)
    plt.plot(*start, 'kx', markersize=10)
    plt.plot(*goal, 'kx', markersize=10)
relaxation = False
# gcs = LinearGCS(regions, edges)
# waypoints, results_dict, best_path = gcs.SolvePath(relaxation)

order = 3
continuity = 1
gcs = BezierGCS(regions, order, continuity, edges)
gcs.addSourceTarget(start, goal)
gcs.setSolver(MosekSolver())
traj, results_dict, best_path = gcs.SolvePath(relaxation)
N = 1000
waypoints = np.zeros((N,2))
n = 0
for t in np.linspace(traj.start_time(), traj.end_time(), N):
    waypoints[n,:] = traj.value(t).reshape(-1)
    n+=1

print("\n")
soln_vid = []
for edge in best_path:
    # print(edge.u().id().get_value(), edge.v().id().get_value(), edge.id().get_value())
    soln_vid.append(edge.u().id().get_value())

soln_vid = [s-1 for s in soln_vid]


print(waypoints.shape)
plot_maze()
plt.plot(*waypoints, 'b')
plt.show()
