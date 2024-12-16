from matplotlib.widgets import Slider
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from queue import PriorityQueue
from tkinter import Tk, messagebox
import time

# Grid3D class defines a 3D grid with obstacles and provides pathfinding and visualization capabilities
class Grid3D:
    def __init__(self, grid_size, obstacle_count, start=None, end=None):
        """
        Initializes the 3D grid with given size and obstacle count.
        If start or end points are not provided, they are randomly generated.
        """
        self.grid_size = grid_size
        self.grid = [[[0 for _ in range(grid_size)] for _ in range(grid_size)] for _ in range(grid_size)]
        self.obstacles = self.generate_obstacles(obstacle_count)
        self.start, self.end = self.validate_positions(start, end)

    def generate_obstacles(self, obstacle_count):
        """
        Randomly generates a set of obstacles within the grid, ensuring they do not exceed the maximum allowed count.
        """
        max_obstacles = self.grid_size ** 3 - 2  # Exclude start and end points
        if obstacle_count > max_obstacles:
            raise ValueError(f"Too many obstacles! Maximum allowed for this grid is {max_obstacles}.")

        obstacles = set()
        while len(obstacles) < obstacle_count:
            x = random.randint(0, self.grid_size - 1)
            y = random.randint(0, self.grid_size - 1)
            z = random.randint(0, self.grid_size - 1)
            obstacles.add((x, y, z))
        return obstacles

    def validate_positions(self, start, end):
        """
        Validates and adjusts start and end positions. Ensures they are not in obstacle positions and not the same.
        """
        if not start or not end:
            return self.randomize_positions()

        if start in self.obstacles:
            self.obstacles.remove(start)
        if end in self.obstacles:
            self.obstacles.remove(end)
        if start == end:
            raise ValueError("Start and End points cannot be the same.")
        return start, end

    def randomize_positions(self):
        """
        Randomly generates valid start and end positions within the grid.
        """
        while True:
            start = (random.randint(0, self.grid_size - 1), random.randint(0, self.grid_size - 1), random.randint(0, self.grid_size - 1))
            end = (random.randint(0, self.grid_size - 1), random.randint(0, self.grid_size - 1), random.randint(0, self.grid_size - 1))
            if start != end and start not in self.obstacles and end not in self.obstacles:
                return start, end

    def a_star_search(self, start, end):
        """
        Performs the A* search algorithm to find the shortest path from start to end in the grid.
        """
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])  # Manhattan distance

        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}

        # Initialize g_score and f_score for all nodes
        g_score = { (x, y, z): float('inf') for x in range(self.grid_size) for y in range(self.grid_size) for z in range(self.grid_size) }
        g_score[start] = 0

        f_score = { (x, y, z): float('inf') for x in range(self.grid_size) for y in range(self.grid_size) for z in range(self.grid_size) }
        f_score[start] = heuristic(start, end)

        while not open_set.empty():
            _, current = open_set.get()

            # If we reach the goal, reconstruct and return the path
            if current == end:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                self.show_message("Path Found", f"Shortest path found! Length: {len(path)}.\nPath: {path}")
                return path

            # Explore neighbors
            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, end)
                    open_set.put((f_score[neighbor], neighbor))

        self.show_message("No Path Found", "No path could be found between the start and end points.")
        return []

    def get_neighbors(self, cell):
        """
        Returns all valid neighbors of a given cell, excluding those outside the grid or within obstacles.
        """
        directions = [
            (dx, dy, dz) for dx in [-1, 0, 1] for dy in [-1, 0, 1] for dz in [-1, 0, 1]
            if not (dx == 0 and dy == 0 and dz == 0)
        ]
        neighbors = []
        for dx, dy, dz in directions:
            x, y, z = cell[0] + dx, cell[1] + dy, cell[2] + dz
            if 0 <= x < self.grid_size and 0 <= y < self.grid_size and 0 <= z < self.grid_size and (x, y, z) not in self.obstacles:
                neighbors.append((x, y, z))
        return neighbors

    def show_message(self, title, message):
        """
        Displays a message box with the given title and message.
        """
        root = Tk()
        root.withdraw()
        messagebox.showinfo(title, message)
        root.destroy()

    def visualize(self, path):
        """
        Visualizes the 3D grid, obstacles, start, end, and the path found using A*.
        Includes an opacity slider to adjust obstacle transparency.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        initial_alpha = 0.6  # Initial opacity value

        # Plot start and end points
        ax.scatter(*self.start, c="green", s=100, label="Start")
        ax.scatter(*self.end, c="red", s=100, label="End")

        # Plot obstacles with adjustable opacity
        obs_x, obs_y, obs_z = zip(*self.obstacles) if self.obstacles else ([], [], [])
        obstacle_plot = ax.scatter(obs_x, obs_y, obs_z, c="gray", s=100, label="Obstacle", alpha=initial_alpha)

        # Add a slider to control obstacle opacity
        slider_ax = fig.add_axes([0.2, 0.02, 0.6, 0.03], facecolor='lightgoldenrodyellow')
        opacity_slider = Slider(slider_ax, 'Opacity', 0.1, 1.0, valinit=initial_alpha, valstep=0.05)

        def update_opacity(val):
            alpha = opacity_slider.val
            obstacle_plot.set_alpha(alpha)
            fig.canvas.draw_idle()

        opacity_slider.on_changed(update_opacity)

        # Visualize the path
        if path:
            path_length = len(path)
            pause_time = max(0.1, 1 * path_length / 100)

            for i in range(len(path)):
                if i > 0:
                    segment = path[i - 1:i + 1]
                    segment_x, segment_y, segment_z = zip(*segment)
                    ax.plot(segment_x, segment_y, segment_z, c="blue", linewidth=2)
                plt.pause(pause_time)

            print("Drone reached the destination!")
        else:
            print("No path found.")

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        plt.legend()

        plt.show()


# Main function to initialize and run the program
def main():
    try:
        # Get user input for grid size, obstacle count, and start/end points
        grid_size = int(input("Enter grid size (e.g., 10): "))
        obstacle_count = int(input("Enter number of obstacles: "))
        start = tuple(map(int, input("Enter start point as x, y, z (e.g., 0 0 0): ").split()))
        end = tuple(map(int, input("Enter end point as x, y, z (e.g., 9 9 9): ").split()))

        grid3d = Grid3D(grid_size, obstacle_count, start=start, end=end)
        path = grid3d.a_star_search(grid3d.start, grid3d.end)
        grid3d.visualize(path)
    except ValueError as e:
        print(f"Invalid input: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")


# Run the program
if __name__ == "__main__":
    main()
