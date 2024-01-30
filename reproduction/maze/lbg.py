import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.collections import PatchCollection
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, Normalize
from random import choice, randint, seed

# from pydrake.geometry.optimization import HPolyhedron
from pydrake.solvers import MosekSolver

from gcs.bezier import BezierGCS
from gcs.linear import LinearGCS
from models.maze import Maze

from gcs import util

def loadDict(file_path):
    # Read the content of the text file
    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Initialize an empty dictionary
    data_dict = {}

    # Parse each line and populate the dictionary
    for line in lines:
        parts = line.strip().split(':')
        key = int(parts[0])
        values = [float(value) for value in parts[1].split()]
        data_dict[key] = values

    return data_dict


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
lbg_cost = np.loadtxt('../data/maze2d/maze_lbg.txt', delimiter=' ')
lbg_adj = loadDict('../data/maze2d/lbg_adj.txt')
newid_to_state = loadDict('../data/maze2d/newid_to_state.txt')
lbg_adj_cost = loadDict('../data/maze2d/lbg_adj_cost.txt')

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
    # plt.plot(*start, 'kx', markersize=10)
    # plt.plot(*goal, 'kx', markersize=10)

plt.figure()
plot_maze()
# Create a scatter plot with square markers and colors
# plt.scatter(xx, yy, s=10, c=col, cmap='viridis', norm=mpl.colors.LogNorm())
plt.scatter(xx, yy, s=10, c=col, cmap='viridis')
# Add a color bar for reference
cbar = plt.colorbar()
cbar.set_label('LBG cost')
plt.savefig('../data/maze2d/maze_lbg_cost.pdf')

plt.figure()
plot_maze()
ax = plt.gca()
edx = []
edy = []
costs = []
plt_edges = {}
for node in lbg_adj:
    i=0
    for neigh in lbg_adj[node]:
        # Skip zero costs
        # if lbg_adj_cost[node][i] < 1e-3:
        #     i += 1
        #     continue

        # skip one of bidirectional edges
        ed = (node, neigh)
        ef = ed
        ei = ed[::-1]
        if ed in plt_edges or ei in plt_edges:
            continue

        plt_edges[ed] = True

        costs.append(lbg_adj_cost[node][i])
        # costs.append(abs(lbg_adj_cost[node][i]-1))
        i+=1

        edx.append(newid_to_state[node][0])
        edx.append(newid_to_state[int(neigh)][0])
        edy.append(newid_to_state[node][1])
        edy.append(newid_to_state[int(neigh)][1])

        # edx = [newid_to_state[node][0], newid_to_state[int(neigh)][0]]
        # edy = [newid_to_state[node][1], newid_to_state[int(neigh)][1]]
        # col = lbg_adj_cost[node][i]

        # plt.plot(edx, edy, 'b-')
        # plt.plot(edx, edy, c=col, cmap='viridis')

# Create a colormap based on costs
cmap = plt.get_cmap('bwr')
norm = Normalize(vmin=np.min(costs), vmax=np.max(costs))
# colors = cmap(norm(costs))

# Create a LineCollection with colors based on costs
points = np.array([edx, edy]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[::2], points[1::2]], axis=1)
lc = LineCollection(segments, cmap=cmap, norm=norm, linewidth=.5)
lc.set_array(costs)

# Create a figure and axis
# fig, ax = plt.subplots()

# Add the LineCollection to the axis
ax.add_collection(lc)

# Add a colorbar
# cbar = plt.colorbar(lc, ax=ax, orientation='vertical')
# cbar.set_label('Cost')

# Remove margins
ax.margins(x=0, y=0)


plt.savefig('../data/maze2d/maze_lbg.pdf')

plt.xlim([0, 50])
plt.ylim([0, 50])
plt.show()
