from queue import PriorityQueue
import math
from enum import Enum
from typing import Tuple

import networkx as nx

from planning_utils import distance, closest_point


class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)
    SW = (-1, 1, math.sqrt(2))
    NW = (-1, -1, math.sqrt(2))
    SE = (1, 1, math.sqrt(2))
    NE = (1, -1, math.sqrt(2))

    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'
        else:
            return self.name

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = []
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node
    for a in Action:
        newx, newy = x + a.delta[0], y + a.delta[1]
        if within_grid(newx, newy, *(grid.shape)) and grid[newx, newy] == 0:
            valid.append(a)

    return valid


def within_grid(x, y, n1, n2):
    if x < 0 or x>=n1:
        return False
    if y<0 or y>=n2:
        return False
    return True


def a_star_grid(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append((goal, None))
        while branch[n][1] != start:
            path.append(branch[n][1:])
            n = branch[n][1]
        path.append(branch[n][1:])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost


def a_star_graph(graph, h, start, goal):
    """Modified A* to work with NetworkX graphs."""

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                branch_cost = current_cost + cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append((goal, None))
        while branch[n][1] != start:
            path.append(branch[n][1:])
            n = branch[n][1]
        path.append(branch[n][1:])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost


def heuristic(position, goal_position):
    return distance(position, goal_position)

def astar_graph_wrapper(G: nx.Graph, start: Tuple[float, float], goal: Tuple[float, float]) -> Tuple:
    start_ne_g = closest_point(G, start)
    goal_ne_g = closest_point(G, goal)
    print(start_ne_g)
    print(goal_ne_g)
    path, cost = a_star_graph(G, heuristic, start_ne_g, goal_ne_g)
    return start_ne_g, goal_ne_g, path, cost