from collections import deque


def BFS(robot, grid):
    # Performs Breadth-First Search (BFS) to find a path for the robot
    grid_seen = (grid.grid > -1) * grid.explored - 1  # Keeps track of seen positions in the grid
    grid_seen[robot.x, robot.y] = 1  # Marks the starting position as open

    opened = deque()  # Queue to store nodes to be explored
    # Adds the starting node to the queue with initial exploration level and iteration count
    opened.append((robot.x, robot.y, 0, 0))

    previous = dict()  # Dictionary to store previous nodes for path reconstruction
    # Sets the previous node of the starting position to None
    previous[(robot.x, robot.y)] = None

    if not find_path(grid, grid_seen, opened, previous):
        # If no path is found, returns an empty list
        return []

    # Finds the position with the maximum exploration level in the queue
    x, y, _, _ = max(opened, key=lambda n: n[2])
    # Reconstructs the path using the previous dictionary
    return reconstruct_path(previous, (x, y))


def find_path(grid, grid_seen, opened, previous):
    # Expands nodes in the BFS search until convergence or all nodes are explored
    converged = False
    while not converged and len(opened) > 0:
        # Retrieves and removes the leftmost node from the queue
        node = opened.popleft()

        if expand_node(node, opened, previous, grid_seen, grid):
            # Convergence occurs if an unexplored position is found
            converged = True

            _, _, _, iteration = node
            size = len(opened)

            # Expands nodes in the queue with the same iteration count
            for i in range(size):
                node = opened[i]
                if node[3] > iteration:
                    break
                expand_node(node, opened, previous, grid_seen, grid)
    return converged


def expand_node(node, opened, previous, grid_seen, grid):
    # Expands a node by considering its neighboring positions
    x, y, _, iteration = node
    grid_seen[x, y] = 2  # Marks the current position as close
    found_unexplored = False

    for x_open, y_open in [(x - 1, y), (x, y + 1), (x + 1, y), (x, y - 1)]:
        # Checks the neighboring positions
        if grid_seen[x_open, y_open] != 0:
            continue
        # Calculates the exploration level of the neighboring position
        expl_lvl = grid.calculate_exploration_level(x_open, y_open)

        previous[(x_open, y_open)] = (x, y)  # Sets the previous node of the neighboring position
        # Adds the neighboring position to the queue with updated exploration level and iteration count
        opened.append((x_open, y_open, expl_lvl, iteration + 1))
        grid_seen[x_open, y_open] = 1  # Marks the neighboring position as open

        found_unexplored = found_unexplored or expl_lvl > 0  # Updates the flag if an unexplored position is found
    return found_unexplored


def reconstruct_path(previous, goal):
    # Reconstructs the path from the previous nodes
    path = list()
    path.append(goal)

    current = previous[goal]
    while current is not None:
        # previous for start will return None
        path.append(current)
        current = previous[current]
    return path[:-1]  # path without start
