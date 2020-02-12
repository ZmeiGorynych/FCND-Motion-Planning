import networkx as nx
import matplotlib.pyplot as plt


def plot_graph(g: nx.Graph, color: str = "b"):
    for e in g.edges:
        p1 = e[0]
        p2 = e[1]
        plt.plot([p1[1], p2[1]], [p1[0], p2[0]], color + "-")


def plot_path(start_ne, goal_ne, path, color="r"):
    if type(path[0][0]) == tuple:
        path = [p[0] for p in path]
    plt.plot([start_ne[1], path[0][1]], [start_ne[0], path[0][0]], color + "-")
    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i + 1]
        plt.plot([p1[1], p2[1]], [p1[0], p2[0]], color + "-")
    plt.plot([goal_ne[1], path[-1][1]], [goal_ne[0], path[-1][0]], color + "-")
