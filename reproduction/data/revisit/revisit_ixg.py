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

start = np.array([3.1, 0.5])
goal = np.array([2, 3.5])

regions = util.DeserializeRegions('/home/gaussian/cmu_ri_phd/phd_research/INSATxGCS-Planner/comparison/revisit/revisit_regions.csv')
edges = util.DeserializeEdges('/home/gaussian/cmu_ri_phd/phd_research/INSATxGCS-Planner/comparison/revisit/revisit_edges.csv')


relaxation = False
# gcs = LinearGCS(regions, edges)
# waypoints, results_dict, best_path = gcs.SolvePath(relaxation)

order = 7
continuity = 4
gcs = BezierGCS(regions, order, continuity, edges)
gcs.addSourceTarget(start, goal)

gcs.addTimeCost(1e-3)
gcs.addPathLengthCost(1)
gcs.addVelocityLimits(-10 * np.ones(2), 10 * np.ones(2))
regularization = 1e-3
gcs.addDerivativeRegularization(regularization, regularization, 2)
gcs.addDerivativeRegularization(regularization, regularization, 3)
gcs.addDerivativeRegularization(regularization, regularization, 4)

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
print(soln_vid)


print(waypoints.shape)

vertices_bl = [[0, 0], [0, 1.5], [3, 1], [3, 0]]
vertices_br = [[3, 0], [3, 1], [4, 2], [6, 0]]
vertices_tl = [[0, 2.5], [0, 4], [3, 4], [3, 3]]
vertices_tr = [[3, 3], [3, 4], [6, 4], [6, 0]]
obs = [[0, 1.5], [3, 1], [4, 2], [3, 3], [0, 2.5]]

plt.figure()
plt.fill([vertex[0] for vertex in obs], [vertex[1] for vertex in obs], color='k', alpha=0.5)
plt.fill([vertex[0] for vertex in vertices_bl], [vertex[1] for vertex in vertices_bl], alpha=0.3)
plt.fill([vertex[0] for vertex in vertices_br], [vertex[1] for vertex in vertices_br], alpha=0.3)
plt.fill([vertex[0] for vertex in vertices_tr], [vertex[1] for vertex in vertices_tr], alpha=0.3)
plt.fill([vertex[0] for vertex in vertices_tl], [vertex[1] for vertex in vertices_tl], alpha=0.3)

plt.xlim([0, 6])
plt.ylim([0, 4])
# plt.plot(*waypoints, 'b')
plt.plot(start[0], start[1], 'r.', markersize=15)
plt.plot(goal[0], goal[1], 'g.', markersize=15)
plt.plot(waypoints[:, 0], waypoints[:, 1], 'b')

# plt.savefig('revisit.pdf', bbox_inches='tight', pad_inches=0)
plt.savefig('revisit_w_traj.pdf', bbox_inches='tight', pad_inches=0)
plt.show()
