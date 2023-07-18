from src.bfs import BFS


class Robot:
    def __init__(self, x: int, y: int):
        self.x = x
        self.y = y
        self.carry_maze = False
        self.path = []

    def transportation_move(self, grid):
        # Check if the robot has predetermined path
        if len(self.path) == 0:
            if grid.grid[self.x, self.y] == 2:
                # If there is a maze at the current position, the robot will pick up maze
                self.carry_maze = True
                grid.grid[self.x, self.y] = 0
                return True
            elif grid.grid[self.x, self.y] == 1:
                # If the robot gets to the start, drop the maze
                self.carry_maze = False
                return True
            else:
                raise ValueError(f"No maze in position {self.x}, {self.y}!")
        # moving the robot to the next position
        self.x, self.y = self.path.pop()
        self.explore(grid)
        return False

    def explore(self, grid):
        # Marks the current position as explored
        grid.explored[self.x, self.y] = True
        grid_shape = grid.grid.shape

        # Explores adjacent positions in the grid
        for i in [1, -1]:
            if 0 <= self.x + i < grid_shape[0]:
                grid.explored[self.x + i, self.y] = True
            if 0 <= self.y + i < grid_shape[1]:
                grid.explored[self.x, self.y + i] = True
        return

    def exploration_move(self, grid):
        # Checks if the robot has no more steps to take or the exploration level of the destination position become zero
        if len(self.path) == 0 or grid.calculate_exploration_level(*self.path[0]) == 0:
            # Uses the Breadth-First Search algorithm to find a new path for exploration
            self.path = BFS(self, grid)
        if len(self.path) == 0:
            # Zero path indicates that there is nothing more to explore
            return False

        # Moves the robot to the next position in the path
        self.x, self.y = self.path.pop()
        return True

