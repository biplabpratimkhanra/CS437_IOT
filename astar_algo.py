import picar_4wd as fc
import sys
import signal
import numpy as np
import time
from queue import PriorityQueue
from car_mapping import Car

class Node:
    def __init__(self, parent_node=None, position=None):
        self.parent_node = parent_node
        self.position = position

        # initializing these values for a-star calculation
        self.g = 0
        self.h = 0

# this function is used to return the path that we've found
def backtrace(parent, end):
    path = [end]
    # for every previous node in parent
    while (path[-1] in parent):
        # append the value -- which is the parent itself
        path.append(parent[path[-1]])

    path.reverse()
    return path

# helper function for calculating manhattan distance heuristic
def manhattan_heuristic(a_coord, b_coord):
    res_x = abs(a_coord[0] - b_coord[0])
    res_y = abs(a_coord[1] - b_coord[1])
   
    return res_x + res_y
# compute all the neighbors of the current node
def compute_neighbors(node):
    node_x_coord = node[0]
    node_y_coord = node[1]

    # append the left, right, top, and bottom neighbors
    neighbors = []
    neighbors.append((node_x_coord - 1, node_y_coord))
    neighbors.append((node_x_coord + 1, node_y_coord))
    neighbors.append((node_x_coord, node_y_coord - 1))
    neighbors.append((node_x_coord, node_y_coord + 1))

    return neighbors

def astar_search(maze, start_coord, end_coord):
    path = []
    # queue used for a* search
    queue = []

    # keep track of already visited points
    visited = []

    # the Manhattan distance is the a* heuristic function h(x) & the path length from a specific state to the start state is the g(x) function
    # the start coord will not have a parent node, and the h, g, and f vals are all set to 0
    start_coord.g = start_coord.h = start_coord.f = 0
    start_tuple = tuple(start_coord)
    start_node = Node(None, start_tuple)

    end_coord.g = end_coord.h = end_coord.f = 0
    end_tuple = tuple(end_coord)
    end_node = Node(None, end_tuple)

    initial_dist = 0

    # dictionary to store distances at their current node values
    visited.append(start_coord)

    # keep a dictionary for parent nodes
    parent = {}

    # keep a dictionary for all g values
    g_val_dict = {}
    g_val_dict[start_coord] = 0

    # traverse maze, push current element on queue, then process neighbors
    while(queue):
        # node = heappop(queue)
        # node will hold the tuple of values -- g, h, and f
        node = queue[1]

        if(node == end_coord):
            break

        neighbor_nodes = compute_neighbors(node)

        for neighbor in neighbor_nodes:
            if neighbor not in g_val_dict:
                # update the value of each neighbor inside of g_val_dict
                g_val_dict[neighbor] = g_val_dict[node] + 1

                # f(n) = h(n) + g(n) where h(n) is the heuristic, g(n) is the length from this state to the start-state
                function_val = manhattan_heuristic(neighbor, end_coord) + (g_val_dict[neighbor])

                # pushing in a function f(x) val and the corresponding neighbor coordinate
                heapq.heappush(queue, (function_val, neighbor))

                parent[neighbor] = node

    path = backtrace(parent, end_coord)
    return path