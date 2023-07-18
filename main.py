import numpy as np
from src.maincontroler import load_from_file
import pygame
import argparse


parser = argparse.ArgumentParser()
# Arguments
parser.add_argument("--path", default="grids/grid_big2.txt", type=str, help="Path to the grid.")
parser.add_argument("--width", default=1000, type=int, help="Width of the window.")
parser.add_argument("--time_delay", default=1, type=int, help="Time delay between steps.")
parser.add_argument("--verbose", default=False, action="store_true", help="Printing steps to the console.")


class PygameDrawer:
    # Class for handling the visualization of the grid using Pygame
    RED = (255, 0, 0)
    GREEN = (0, 255, 0)
    BLUE = (18, 203, 196)
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    PURPLE = (139, 0, 139)
    GREY = (128, 128, 128)

    def __init__(self, grid_shape, args):
        self.time_delay = args.time_delay

        self.width = args.width
        self.height = round(grid_shape[0] / grid_shape[1] * self.width)
        # Initializing screen
        self.grid = np.zeros((grid_shape[0], grid_shape[1], 3))
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Maze Algorithm")
        self.surface = pygame.Surface((grid_shape[1], grid_shape[0]))

    def draw_grid(self, grid, robots, explored=False, check_quit=True):
        # Draws the grid on the Pygame screen.
        if check_quit:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.quit()
        pygame.time.delay(self.time_delay)

        self.grid[grid.grid == -1, :] = self.BLACK
        self.grid[grid.grid == 0, :] = self.WHITE
        self.grid[grid.grid == 1, :] = self.BLUE
        self.grid[grid.grid == 2, :] = self.PURPLE
        if explored:
            self.grid[np.logical_not(grid.explored), :] = self.GREY
        for rob in robots:
            self.grid[rob.x, rob.y, :] = self.RED if rob.carry_maze else self.GREEN

        # Draw the array onto the surface
        pygame.surfarray.blit_array(self.surface, self.grid.transpose((1, 0, 2)))
        # Transform the surface to screen size
        surface = pygame.transform.scale(self.surface, (self.width, self.height))
        self.screen.fill((0, 0, 0))
        # Blit the transformed surface onto the screen
        self.screen.blit(surface, (0, 0))
        pygame.display.update()

    def quit(self):
        # Quits the Pygame app
        pygame.quit()


def main(args):
    # Load the grid and robots from the specified file
    mc = load_from_file(args.path)
    # Create a PygameDrawer object to visualize the grid and robots
    pg_drawer = PygameDrawer(mc.grid.grid.shape, args)
    # Initialize flags for exploration and transportation
    running = True
    exploration = False
    transportation = False

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    # Perform exploration if not already done
                    if not exploration:
                        mc.do_exploration(verbose=False, pg_drawer=pg_drawer)
                        exploration = True
                    # Perform transportation if exploration is done but transportation is not
                    elif not transportation:
                        mc.do_transportation(verbose=False, pg_drawer=pg_drawer)
                        transportation = True
                    # Exit the loop if both exploration and transportation are done
                    else:
                        running = False
        # Draw the grid and robots on the Pygame screen
        pg_drawer.draw_grid(mc.grid, mc.robots, exploration or transportation, check_quit=False)

    # Quit the Pygame application
    pg_drawer.quit()
    # Print the final result of the maze algorithm
    mc.print_result()


if __name__ == "__main__":
    args = parser.parse_args([] if "__file__" not in globals() else None)
    main(args)
