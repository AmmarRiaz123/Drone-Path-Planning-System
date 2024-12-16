# Grid3D Pathfinding Project

This project implements a **3D grid-based pathfinding system** using the **A\* search algorithm**. The application generates obstacles, finds a path from a start point to an endpoint, and visualizes the 3D grid along with the obstacles and the computed path.

---

## Features
- **3D Grid Representation**: Define a 3D grid of customizable size.
- **Random Obstacle Placement**: Randomly places obstacles in the grid while ensuring that the start and end points are valid.
- **A\* Search Algorithm**: Finds the shortest path from the start to the end point using the heuristic of Manhattan distance.
- **Interactive Visualization**:
  - Displays the 3D grid with start, end points, obstacles, and the computed path.
  - Includes a slider to adjust obstacle transparency during visualization.
- **Error Handling**: Ensures valid input for grid size, obstacle count, and start/end points. Displays appropriate messages if no path is found.

---

## Requirements
- **Python**: 3.7 or later
- **Libraries**:
  - `matplotlib`
  - `mpl_toolkits.mplot3d`
  - `tkinter`

Install the required libraries using pip:
```bash
pip install matplotlib
```

---

## How to Use

### Running the Program
1. Clone this repository:
   ```bash
   git clone <repository-url>
   ```
2. Navigate to the project directory:
   ```bash
   cd Grid3D-Pathfinding
   ```
3. Run the Python script:
   ```bash
   python grid3d_pathfinding.py
   ```

### Input Parameters
When prompted, provide the following inputs:
1. **Grid Size**: Size of the cubic 3D grid (e.g., `10` for a 10x10x10 grid).
2. **Number of Obstacles**: Total number of random obstacles to generate.
3. **Start Point**: Coordinates of the start point as `x y z` (e.g., `0 0 0`).
4. **End Point**: Coordinates of the end point as `x y z` (e.g., `9 9 9`).

### Output
- If a path is found:
  - Displays the shortest path and its length.
  - Visualizes the 3D grid, including the obstacles and the computed path.
- If no path is found:
  - Displays a message indicating that no path exists between the start and end points.

---

## Code Overview

### `Grid3D` Class
#### Methods:
- `__init__(grid_size, obstacle_count, start, end)`:
  - Initializes the grid, places obstacles, and validates positions.
- `generate_obstacles(obstacle_count)`:
  - Generates random obstacles within the grid.
- `validate_positions(start, end)`:
  - Ensures that start and end points are valid (not overlapping with obstacles).
- `a_star_search(start, end)`:
  - Implements the A\* search algorithm to compute the shortest path.
- `get_neighbors(cell)`:
  - Retrieves valid neighboring cells of a given cell.
- `show_message(title, message)`:
  - Displays messages using a Tkinter popup.
- `visualize(path)`:
  - Visualizes the grid, obstacles, and the path using Matplotlib.

### Example Output
#### Visualization:
- Start point: Green dot
- End point: Red dot
- Obstacles: Gray dots (opacity adjustable via slider)
- Path: Blue line connecting points

---

## Error Handling
- Ensures valid inputs for grid size, obstacle count, and start/end points.
- Handles cases where the number of obstacles exceeds the maximum allowable count.
- Displays appropriate error messages for invalid inputs or unreachable endpoints.

---

## Future Enhancements
- Add support for non-Manhattan distance heuristics (e.g., Euclidean).
- Allow loading grid configurations from a file.
- Optimize A\* implementation for larger grids.

---

## License
No license specified for this project.

---

## Contributing
Contributions are welcome! If you encounter any issues or have suggestions for improvements, please open an issue or submit a pull request.

---

## Contact
Feel free to reach out with questions or suggestions:
- **Email**: futuristicgamer2321@gmail.com


