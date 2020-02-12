from unittest import TestCase
import numpy as np
import copy
from skimage.morphology import medial_axis
from skimage.util import invert

from medial_graph import skeleton_to_graph, simplify_graph
from grid import create_grid_and_edges, create_grid
from planning_utils import closest_point
from astar import a_star_graph, heuristic, astar_graph_wrapper


class TestSkeleton_to_graph(TestCase):
    def test_skeleton_to_graph(self):
        # This is the same obstacle data from the previous lesson.
        filename = "../colliders.csv"
        data = np.loadtxt(filename, delimiter=",", dtype="Float64", skiprows=2)
        start_ne = (25, 100)
        goal_ne = (750.0, 370.0)
        # Static drone altitude (metres)
        drone_altitude = 5
        safety_distance = 3
        # This is now the routine using Voronoi
        grid, _, _ = create_grid(data, drone_altitude, safety_distance)
        skeleton = medial_axis(invert(grid), return_distance=False)
        G = skeleton_to_graph(skeleton)
        G2 = simplify_graph(copy.deepcopy(G))

        start_ne_g, goal_ne_g, path, cost = astar_graph_wrapper(G, start_ne, goal_ne)
        start_ne_g2, goal_ne_g2, path2, cost2 = astar_graph_wrapper(
            G2, start_ne, goal_ne
        )

        print(len(path), cost, len(path2), cost2)
        print("done!")
