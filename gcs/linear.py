import numpy as np
import pydot
import time

from pydrake.geometry.optimization import (
    Point,
)
from pydrake.solvers import (
    Binding,
    Constraint,
    Cost,
    L2NormCost,
    LinearConstraint,
)

from gcs.base import BaseGCS

class LinearGCS(BaseGCS):
    def __init__(self, regions, edges=None, path_weights=None, full_dim_overlap=False):
        BaseGCS.__init__(self, regions)

        if path_weights is None:
            path_weights = np.ones(self.dimension)
        elif isinstance(path_weights, float) or isinstance(path_weights, int):
            path_weights = path_weights * np.ones(self.dimension)
        assert len(path_weights) == self.dimension

        self.edge_cost = L2NormCost(
            np.hstack((np.diag(-path_weights), np.diag(path_weights))),
            np.zeros(self.dimension))

        for i, r in enumerate(self.regions):
            vertex = self.gcs.AddVertex(r, name = self.names[i] if not self.names is None else '')
            # print('added vertex with id: ', vertex.id().get_value())


        if edges is None:
            if full_dim_overlap:
                edges = self.findEdgesViaFullDimensionOverlaps()
            else:
                edges = self.findEdgesViaOverlaps()
        else:
            vertices = self.gcs.Vertices()
            target_idx = edges[-1][1]
            self.target = vertices[target_idx]


        vertices = self.gcs.Vertices()
        for ii, jj in edges:
            u = vertices[ii]
            v = vertices[jj]
            edge = self.gcs.AddEdge(u, v, f"({u.name()}, {v.name()})")

            edge_length = edge.AddCost(Binding[Cost](
                self.edge_cost, np.append(u.x(), v.x())))[1]

            # Constrain point in v to be in u
            edge.AddConstraint(Binding[Constraint](
                LinearConstraint(u.set().A(),
                                 -np.inf*np.ones(len(u.set().b())),
                                 u.set().b()),
                v.x()))

    def addSourceTarget(self, source, target, edges=None):
        source_edges, target_edges = super().addSourceTarget(source, target, edges)

        for edge in source_edges:
            for jj in range(self.dimension):
                edge.AddConstraint(edge.xu()[jj] == edge.xv()[jj])

        for edge in target_edges:
            edge.AddCost(Binding[Cost](
                self.edge_cost, np.append(edge.xu(), edge.xv())))

    def addSource(self, source):
        source_edges = super().addSource(source)

        for edge in source_edges:
            for jj in range(self.dimension):
                edge.AddConstraint(edge.xu()[jj] == edge.xv()[jj])

    def addTarget(self, target):
        target_edges = super().addTarget(target)

        for edge in target_edges:
            edge.AddCost(Binding[Cost](
                self.edge_cost, np.append(edge.xu(), edge.xv())))

    def getVertex(self, cset):
        vertices = self.gcs.Vertices()
        for v in vertices:
            if v.set() == cset:
                return v
        return None

    def SolvePath(self, rounding=False, verbose=False, preprocessing=False):
        best_path, best_result, results_dict = self.solveGCS(
            rounding, preprocessing, verbose)

        if best_path is None:
            return None, best_result, results_dict, None

        # Extract trajectory
        waypoints = np.empty((self.dimension, 0))
        for edge in best_path:
            new_waypoint = best_result.GetSolution(edge.xv())
            waypoints = np.concatenate(
                [waypoints, np.expand_dims(new_waypoint, 1)], axis=1)

        return waypoints, best_result, results_dict, best_path
