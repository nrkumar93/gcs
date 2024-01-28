import os
import numpy as np
import matplotlib.pyplot as plt
from random import choice, randint, seed

from pydrake.solvers import MosekSolver
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
# gcs.addSourceTarget(start, goal)
# gcs.setSolver(MosekSolver())
# waypoints, results_dict, best_path = gcs.SolvePath(relaxation)

# print("\n")
# soln_vid = []
# for edge in best_path:
#     # print(edge.u().id().get_value(), edge.v().id().get_value(), edge.id().get_value())
#     soln_vid.append(edge.u().id().get_value())

# soln_vid = [s-1 for s in soln_vid]
# print(soln_vid, sep=", ")


# corres = []
# for i, w in enumerate(waypoints):
#     for j, r in enumerate(regions):
#         print(w)
#         if r.PointInSet(w):
#             corres.append([i, j])

# print(corres)

# print(waypoints.shape)
opt_soln = np.loadtxt('./data/opt_soln.txt', delimiter=',')
# opt_soln = np.loadtxt('./data/opt_soln.txt')
print(opt_soln.T.shape)

# find the GCS ID path
gcs_path = []
for wp in opt_soln:
    rid = 1
    for r in regions:
        if r.PointInSet(wp):
            if len(gcs_path)>0 and gcs_path[-1] == rid:
                break
            gcs_path.append(rid)
            break
        rid+=1

print(gcs_path)

# viz path
# path = [7699,1,51,52,2,3,4,54,104,154,204,203,202,252,302,301,351,401,451,501,551,552,553,554,504,505,506,556,557,507,457,407,406,456,455,405,355,356,306,305,304,303,253,254,255,205,206,256,257,207,157,156,155,105,106,56,57,7,8,9,59,58,108,109,110,111,112,62,61,11,12,13,63,64,14,15,16,66,116,115,114,164,163,162,161,211,210,260,259,209,208,258,308,358,408,409,410,411,461,462,512,513,463,464,414,415,416,466,516,517,518,468,467,417,418,419,369,319,320,270,271,321,322,372,422,472,471,470,520,570,571,572,622,623,624,574,524,525,575,576,626,676,726,776,777,827,877,878,828,829,830,831,881,882,932,982,1032,1033,1083,1133,1134,1135,1185,1184,1183,1233,1283,1333,1332,1382,1383,1433,1483,1533,1583,1584,1534,1535,1485,1435,1385,1386,1436,1486,1487,1537,1587,1586,1585,1635,1685,1735,1785,1835,1885,1935,1936,1937,1987,2037,2087,2086,2136,2186,2185,2235,2285,2286,2287,2237,2238,2239,2289,2290,2340,2341,2342,2343,2344,2294,2244,2245,2195,2196,2197,2247,2246,2296,2346,2396,2446,2496,2497,2447,2448,2449,2499,2500,7702]
# xx = []
# yy = []
# for vtx in path:
#     if vtx > len(regions):
#         continue
#     ctr = regions[int(vtx)-1].ChebyshevCenter()
#     xx.append(ctr[0])
#     yy.append(ctr[1])





plot_maze()
# plt.plot(*waypoints, 'b')
plt.plot(*opt_soln.T, 'r')
# plt.plot(xx, yy, 'b')

region_centers = []
for cell in gcs_path:
    r = regions[cell-1]
    ctrs = r.ChebyshevCenter()
    region_centers.append(ctrs)

for ctrs in region_centers:
    plt.scatter(ctrs[0], ctrs[1], s=10, marker='s', alpha=0.5, edgecolors='green')

plt.savefig('./data/play_with_maze.pdf')
plt.show()
