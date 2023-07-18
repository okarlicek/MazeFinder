import numpy as np


class Grid:
    BACK_ENCODING = {0: '.', -1: '#', 1: 'S', 2: 'B'}

    def __init__(self, grid: np.array):
        self.grid = grid
        self.n_mazes = (self.grid == 2).sum()  # Counts the number of mazes (value 2) in the grid

        # Finds the start position (value 1) in the grid
        start_index = (self.grid == 1).argmax()
        self.start = (start_index // self.grid.shape[1], start_index % self.grid.shape[1])

        # Initializes the explored array with the same shape as the grid
        self.explored = np.full_like(grid, False)
        # Marks the corner points of the grid as explored
        self.explored[[0, 0, -1, -1], [0, -1, 0, -1]] = True

    def print(self, robots: list = None, only_explored: bool = False):
        # Creates a grid_print list based on the conditions
        if only_explored:
            grid_print = [[self.BACK_ENCODING[self.grid[i, j]] if self.explored[i, j] else 'x'
                           for j in range(self.grid.shape[1])]
                          for i in range(self.grid.shape[0])]
        else:
            grid_print = [[self.BACK_ENCODING[self.grid[i, j]]
                           for j in range(self.grid.shape[1])]
                          for i in range(self.grid.shape[0])]
        # Updates the grid_print with robot positions
        if robots is not None:
            for rob in robots:
                grid_print[rob.x][rob.y] = 'R'
        # Prints each line of the grid
        for line in grid_print:
            print(''.join(line))
        print()
        return

    def calculate_exploration_level(self, x: int, y: int) -> int:
        # Calculates the number of unexplored positions adjacent to the given (x, y) position
        pos_x, pos_y = [], []

        # Checks the adjacent positions in the grid
        for i in [1, -1]:
            if 0 <= x + i < self.grid.shape[0]:
                pos_x.append(x + i)
                pos_y.append(y)
            if 0 <= y + i < self.grid.shape[1]:
                pos_x.append(x)
                pos_y.append(y + i)
        # Returns the number of unexplored positions
        return (1 - self.explored[pos_x, pos_y]).sum()

    def validate_move(self, x, y):
        # Checks if the move to the given (x, y) position is valid (not a wall)
        return self.grid[x, y] > -1

