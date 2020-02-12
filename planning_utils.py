import numpy as np


def closest_point(graph, current_point):
    """
    Compute the closest point in the `graph`
    to the `current_point`.
    """
    closest_point = None
    dist = float('inf')
    for p in graph.nodes:
        if closest_point == None:
            closest_point = p
        d = distance(current_point, p)
        if d < dist:
            closest_point = p
            dist = d
    return closest_point


def distance(p, q):
    return np.linalg.norm(np.array(p) - np.array(q))