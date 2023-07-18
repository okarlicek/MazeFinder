from src.robot import Robot
from src.grid import Grid
from src.astar import AStar
import numpy as np
import scipy

MAX_EXPLORATION_STEPS = 10_000

ENCODING = {
    '.': 0,   # Nothing
    ' ': 0,   # Nothing
    '#': -1,  # Wall
    'S': 1,   # Start place
    'B': 2    # Maze
}


class MainController:
    def __init__(self, grid: Grid, robots: list[Robot]):
        # Initializes the MainController object with a grid and a list of robots
        self.grid = grid
        self.robots = robots
        self.exploration_steps = 0
        self.transportation_steps = 0
        for robot in self.robots:
            # Initial exploration
            robot.explore(grid)
        self.mazes_found = (self.grid.grid * self.grid.explored == 2).sum()
        self.start_found = False

    def do_exploration(self, verbose=False, pg_drawer=None):
        # Performs the exploration phase
        explore = True  # Checking if there is something to explore

        while explore:
            # Exploration round
            explore = self.exploration_round()
            self.exploration_steps += 1
            # Checking for founded mazes and start
            self.mazes_found = (self.grid.grid * self.grid.explored == 2).sum()
            self.start_found = (self.grid.grid * self.grid.explored == 1).sum() > 0

            if self.exploration_steps > MAX_EXPLORATION_STEPS:
                # Not wanting infinite loop
                break
            if verbose:
                self.grid.print(self.robots, True)
            if pg_drawer is not None:
                pg_drawer.draw_grid(self.grid, self.robots, True)

    def exploration_round(self):
        # Performs one round of exploration for each robot
        for r in self.robots:
            if not r.exploration_move(self.grid):
                # When there is nothing to explore
                return False
            r.explore(self.grid)
        return True

    def do_transportation(self, verbose=False, pg_drawer=None):
        # Performs the transportation phase if the start place is found
        if not self.start_found:
            return None

        # Position of the founded mazes
        mazes = np.nonzero(self.grid.grid * self.grid.explored == 2)
        mazes = [(mazes[0][i], mazes[1][i]) for i in range(len(mazes[0]))]

        # Initial maze assignment
        self.assign_mazes(mazes)

        while len(self.robots) > 0:
            # Do transportation round
            self.transportation_round(mazes)
            self.transportation_steps += 1
            if verbose:
                self.grid.print(self.robots, True)
            if pg_drawer is not None:
                pg_drawer.draw_grid(self.grid, self.robots, True)

    def assign_mazes(self, mazes):
        # Assign mazes to robots based on the distances between them.
        robots_pos = [(r.x, r.y) for r in self.robots]
        distances = np.zeros((len(robots_pos), len(mazes)))

        # Calculate distances between robots and mazes
        for i in range(len(robots_pos)):
            for j in range(len(mazes)):
                distances[i, j] = abs(robots_pos[i][0] - mazes[j][0]) + abs(robots_pos[i][1] - mazes[j][1])

        to_remove = []
        rob_idx, maze_idx = scipy.optimize.linear_sum_assignment(distances)

        # Assign mazes to robots and remove assigned mazes from the list
        for rob_i, maze_i in zip(rob_idx, maze_idx):
            to_remove.append(mazes[maze_i])
            self.robots[rob_i].path = AStar((self.robots[rob_i].x, self.robots[rob_i].y), mazes[maze_i], self.grid)
        for m in to_remove:
            mazes.remove(m)

    def transportation_round(self, mazes):
        # Performs one round of transportation for each robot
        to_remove = []
        for rob in self.robots:
            # Do transportation move
            if rob.transportation_move(self.grid):
                if rob.carry_maze:
                    # When carrying maze, find path to the beginning
                    rob.path = AStar((rob.x, rob.y), self.grid.start, self.grid)
                else:
                    if len(mazes) == 0:
                        # When no mazes are left, remove the robot
                        to_remove.append(rob)
                        continue
                    else:
                        # Find the nearest maze
                        maze = min(mazes, key=lambda m: abs(m[0] - rob.x) + abs(m[1] - rob.y))
                        rob.path = AStar((rob.x, rob.y), maze, self.grid)
                        mazes.remove(maze)
        for rob in to_remove:
            self.robots.remove(rob)

    def print_result(self):
        # Prints the result of the exploration and transportation phases
        if not self.start_found:
            print(f"Start was not found in {self.exploration_steps} exploration steps!")
            return
        print("Exploration steps: ", self.exploration_steps)
        print("Transportation steps: ", self.transportation_steps)
        print("Total steps: ", self.exploration_steps + self.transportation_steps)
        carried_mazes = self.grid.n_mazes - (self.grid.grid == 2).sum()
        print(f"Transported {carried_mazes} out of {self.grid.n_mazes}")


def instantiate_robot(robots: list[Robot], x: int, y: int):
    # Creates a new Robot object and adds it to the list of robots.
    robots.append(Robot(x, y))
    return 0


def load_from_file(path: str) -> MainController:
    # Loads the grid and robots from a file and creates a MainController
    grid = []
    robots = []
    with open(path) as file:
        for x, line in enumerate(file):
            grid_line = [instantiate_robot(robots, x, y) if ch == 'R' else ENCODING[ch]
                         for y, ch in enumerate(line.strip())]
            grid.append(grid_line)
    return MainController(Grid(np.array(grid)), robots)
