import numpy as np


def AStar(start, goal, grid):
    # Performs the A* algorithm to find a path from the start position to the goal position
    grid_seen = (grid.grid > -1) * grid.explored - 1  # Keeps track of seen positions in the grid
    # Stores the distance from the start position to each position in the grid
    distance = np.zeros_like(grid_seen)

    opened = list()  # List to store nodes to be explored
    previous = dict()  # Dictionary to store previous nodes for path reconstruction
    previous[start] = None  # Sets the previous node of the start position to None

    # Finds a path using A*
    path = find_path(start, goal, distance, grid_seen, opened, previous)
    return path


def find_path(start, goal, distance, grid_seen, opened, previous):
    # Expands nodes in the A* search until convergence or all nodes are explored
    converged = False

    if start == goal:
        converged = True

    if not converged:
        # Expanding first node
        prev = start
        if expand_node(prev, goal, distance, grid_seen, opened, previous):
            return reconstruct_path(previous, goal)

    while len(opened) > 0 and not converged:
        # Expanding until convergence
        to_expand = pick_node(opened, distance)
        if expand_node(to_expand, goal, distance, grid_seen, opened, previous):
            converged = True
            return reconstruct_path(previous, goal)
    return []


def expand_node(node, goal, distance, grid_seen, opened, previous):
    # Expands a node by considering its neighboring positions
    x, y = node[0], node[1]
    grid_seen[x, y] = 2  # Marks the current position as closed

    for x_open, y_open in [(x, y - 1), (x, y + 1), (x - 1, y), (x + 1, y)]:
        if x_open == goal[0] and y_open == goal[1]:
            # We found the goal node
            previous[goal] = (x, y)
            return True

        # Opens a neighboring node
        if open_node(x_open, y_open, node, goal, distance, grid_seen, opened):
            previous[(x_open, y_open)] = (x, y)

    return False


def pick_node(opened, distance):
    # Picks the node from the list with the lowest cost
    to_expand = min(opened, key= lambda n: n[2] + distance[n[0], n[1]])
    opened.remove(to_expand)
    return to_expand


def open_node(x_open, y_open, previous_node, goal, distance, grid_seen, opened):
    # Opens a neighboring node
    dist = distance[previous_node[0], previous_node[1]] + 1
    if grid_seen[x_open, y_open] == 0:
        # Opens a node if it has not been seen before
        opened.append((x_open, y_open, manhattan_heuristic(x_open, y_open, goal)))
        distance[x_open, y_open] = dist
        grid_seen[x_open, y_open] = 1
        return True

    elif grid_seen[x_open, y_open] == 1:
        # Reopens a node if a shorter path to it is found
        if dist < distance[x_open, y_open]:
            distance[x_open, y_open] = dist
            return True
    return False


def reconstruct_path(previous, goal):
    # Reconstructs the path from the previous nodes
    path = list()
    path.append(goal)

    current = previous[goal]
    while current is not None:
        # Appends the current node to the path
        path.append(current)
        current = previous[current]
    return path[:-1]  # Returns the path without the start position


def manhattan_heuristic(x, y, goal):
    # Calculates the Manhattan heuristic value between two positions
    return abs(goal[0] - x) + abs(goal[1] - y)

