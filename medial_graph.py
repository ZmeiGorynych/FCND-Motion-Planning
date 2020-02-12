import math
from typing import List, Tuple

import networkx as nx
import numpy as np

from astar import within_grid
from planning_utils import distance


def skeleton_to_graph(skel: np.ndarray) -> nx.Graph:
    graph = nx.Graph()
    # turn the skeleton into graph, with neighboring points connected
    for i in range(skel.shape[0]):
        for j in range(skel.shape[1]):
            if skel[i, j]:
                for di in (-1, 0, 1):
                    for dj in (-1, 0, 1):
                        if di != 0 or dj != 0:
                            newi = i + di
                            newj = j + dj
                            if within_grid(newi, newj, skel.shape[0], skel.shape[1]):
                                if skel[newi, newj]:
                                    graph.add_edge(
                                        (i, j),
                                        (newi, newj),
                                        weight=math.sqrt(di * di + dj * dj),
                                    )
    return graph


def simplify_graph(graph: nx.Graph, tol: float = 3.0) -> nx.Graph:
    # find all points with zero or more than 2 neighbors, put them into a queue for processing
    major_nodes = [node for node in graph if len(graph.edges(node)) != 2]
    to_process = [node for node in graph if len(graph.edges(node)) != 2]

    # for each point p in a queue
    while len(to_process) > 0:
        if len(to_process) % 100 == 0:
            print(
                "simplifying medial axis graph, nodes left to process:", len(to_process)
            )
        p = to_process.pop()
        p_edges = list(graph.edges(p))
        # we modify graph as we go, so have to be careful
        for e in p_edges:
            q, next_edge = other_end(graph, p, e)
            if q in major_nodes:
                continue
            else:
                in_between = []
                while True:
                    if q in major_nodes:
                        break
                    in_between.append(q)
                    q, next_edge = other_end(graph, q, next_edge)

                    if next_edge is None:
                        break

                    if not all_on_straight_line(
                        np.array(p), np.array(q), in_between, tol
                    ):
                        q = in_between.pop()
                        break

                graph.add_edge(p, q, weight=distance(p, q))
                if q not in major_nodes:
                    to_process.append(q)
                    major_nodes.append(q)

                for pp in in_between:
                    graph.remove_node(pp)

    return graph


def all_on_straight_line(
    p: np.ndarray, q: np.ndarray, in_between: List[Tuple[float, float]], tol: float
):
    d = p - q
    d = d / np.linalg.norm(d)
    c = p[0] * d[1] - p[1] * d[0]
    for x, y in in_between:
        dist = abs(x * d[1] - y * d[0] - c)
        if dist > tol:
            return False
    return True


def other_end(graph, p, edge):
    if p not in edge:
        print("Point must be member of that edge")
    p1 = [pp for pp in edge if pp != p][0]
    other_edges = [ee for ee in graph.edges(p1) if not compare_edges(ee, edge)]
    if len(other_edges) != 1:
        return p1, None
    else:
        return p1, other_edges[0]


def compare_edges(e1, e2):
    if e1 == e2:
        return True
    if (e1[1], e1[0]) == e2:
        return True
    else:
        return False
