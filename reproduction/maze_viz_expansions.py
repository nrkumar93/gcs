import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from random import choice, randint

from models.maze import Maze

from gcs import util

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

regions = util.DeserializeRegions('data/maze2d/maze.csv')
edges = util.DeserializeEdges('data/maze2d/maze_edges.csv')
incom_edge_count = np.loadtxt('/home/gaussian/cmu_ri_phd/phd_research/ixg/logs/insatxgcs_incom_edge_count_1.txt', delimiter=' ')
# incom_edge_count = np.loadtxt('/home/gaussian/cmu_ri_phd/phd_research/ixg/logs/insatxgcs_lb_cost.txt', delimiter=' ')
xx = []
yy = []
col = []
clamp_div = 50
for val in incom_edge_count:
    vtx = val[0]
    if vtx > len(regions):
        continue
    if val[1] > 990:
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
plt.scatter(xx, yy, s=10, c=col, cmap='viridis', norm=mpl.colors.LogNorm())
# Add a color bar for reference
cbar = plt.colorbar()
cbar.set_label('Number of re-expansions')
plt.savefig('./data/maze_num_incom_edges_dup_w1.pdf')
plt.show()
