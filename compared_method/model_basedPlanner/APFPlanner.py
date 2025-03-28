
import math

class ArtificialPotentialField:
    def __init__(self, x_bounds, y_bounds, k, kr, p):
        """
        Initialize the Artificial Potential Field class.
        :param x_bounds: Spatial x-axis range [x_min, x_max].
        :param y_bounds: Spatial y-axis range [y_min, y_max].
        :param k: Attractive force coefficient.
        :param kr: Repulsive force coefficient.
        :param p: Repulsive force range.
        """
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.k = k
        self.kr = kr
        self.p = p
        self.grid = None  # Initialize grid as None
        self.goal = None  # Initialize goal as None
        self.rows = 0
        self.cols = 0
        self.cell_size_x = 0
        self.cell_size_y = 0
        self.potential = None  # Initialize potential as None
        self.used_nodes = set()  # Track used nodes

    def grid_to_space(self, i, j):
        """
        Convert grid indices to spatial coordinates (center of the grid cell).
        :param i: Row index of the grid.
        :param j: Column index of the grid.
        :return: Spatial coordinates (x, y).
        """
        x = self.x_bounds[0] + (j + 0.5) * self.cell_size_x
        y = self.y_bounds[0] + (i + 0.5) * self.cell_size_y
        return (x, y)

    def space_to_grid(self, x, y):
        """
        Convert spatial coordinates to grid indices.
        :param x: Spatial x-coordinate.
        :param y: Spatial y-coordinate.
        :return: Grid indices (i, j).
        """
        i = int((y - self.y_bounds[0]) // self.cell_size_y)
        j = int((x - self.x_bounds[0]) // self.cell_size_x)
        return (i, j)

    def _compute_potential_for_candidates(self, candidates):
        """
        Compute the potential fields only for the candidate nodes.
        :param candidates: List of candidate grid indices [(i1, j1), (i2, j2), ...].
        :return: Dictionary mapping candidate nodes to their potential values.
        """
        potential = {}
        for (i, j) in candidates:
            if self.grid[i][j] == 1:  # Skip obstacle cells
                potential[(i, j)] = float('inf')
                continue
            x, y = self.grid_to_space(i, j)
            # Compute attractive potential
            dx = x - self.goal[0]
            dy = y - self.goal[1]
            attractive = self.k * (dx ** 2 + dy ** 2)
            # Compute repulsive potential
            min_dist = self._min_obstacle_distance(x, y)
            repulsive = 0
            if min_dist < self.p:
                safe_dist = max(min_dist, 1e-6)  # Avoid division by zero
                repulsive = 0.5 * self.kr * (1.0 / safe_dist - 1.0 / self.p) ** 2
            potential[(i, j)] = attractive + repulsive
        return potential

    def _min_obstacle_distance(self, x, y):
        """
        Compute the minimum distance from a point to the nearest obstacle.
        :param x: Spatial x-coordinate.
        :param y: Spatial y-coordinate.
        :return: Minimum distance to the nearest obstacle.
        """
        min_dist = float('inf')
        for i in range(self.rows):
            for j in range(self.cols):
                if self.grid[i][j] == 1:
                    ox, oy = self.grid_to_space(i, j)
                    dist = math.hypot(x - ox, y - oy)
                    if dist < min_dist:
                        min_dist = dist
        return min_dist

    def find_path_nodes(self, grid, robot_pos, goal):
        """
        Find the path nodes starting from the robot's position.
        :param grid: 2D grid map, where 1 represents obstacles and 0 represents free space.
        :param robot_pos: Robot's initial position (x, y).
        :param goal: Goal position (x_g, y_g).
        :return: List of the robot's position and three grid node positions.
        """
        self.grid = grid  # Assign grid as an instance variable
        self.goal = goal  # Assign goal as an instance variable
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0
        self.cell_size_x = (self.x_bounds[1] - self.x_bounds[0]) / self.cols  # Grid cell size in x-direction
        self.cell_size_y = (self.y_bounds[1] - self.y_bounds[0]) / self.rows  # Grid cell size in y-direction
        self.used_nodes = set()  # Reset used nodes

        path = [robot_pos]  # Start with the robot's position
        current_pos = robot_pos
        goal_id = self.space_to_grid(*self.goal)
        for _ in range(3):  # Find three additional nodes
            candidates = self._get_candidate_nodes(current_pos)
            if not candidates:
                break
            # Compute potential only for candidates
            candidate_potentials = self._compute_potential_for_candidates(candidates)
            # Find the node with the minimum potential
            min_node = min(candidate_potentials, key=lambda node: candidate_potentials[node])
            self.used_nodes.add(min_node)  # Mark the node as used
            if goal_id in candidates:
                space_pos = self.goal
                path.append(space_pos)
                break
            else:
                space_pos = self.grid_to_space(*min_node)
                path.append(space_pos)
            current_pos = space_pos
        return path

    def _get_candidate_nodes(self, pos):
        """
        Get candidate nodes (8 neighboring grid cells), excluding used nodes.
        :param pos: Current spatial position (x, y).
        :return: List of candidate grid indices [(i1, j1), (i2, j2), ...].
        """
        i, j = self.space_to_grid(*pos)
        candidates = [
            (i - 1, j - 1), (i - 1, j), (i - 1, j + 1),
            (i, j - 1),                 (i, j + 1),
            (i + 1, j - 1), (i + 1, j), (i + 1, j + 1)
        ]
        valid = []
        for ni, nj in candidates:
            if 0 <= ni < self.rows and 0 <= nj < self.cols:
                if self.grid[ni][nj] == 0 and (ni, nj) not in self.used_nodes:  # Only consider free and unused space
                    valid.append((ni, nj))
        return valid

if __name__ == '__main__':
    # Define the grid map
    grid = [
        [0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0]
    ]

    # Define spatial bounds [x_min, x_max], [y_min, y_max]
    x_bounds = [0, 1]
    y_bounds = [0, 1]

    # Define the goal position
    goal = (0.9, 0.8)

    # Define potential field parameters
    k = 1.0  # Attractive coefficient
    kr = 1.0  # Repulsive coefficient
    p = 0.2  # Repulsive range

    # Initialize the Artificial Potential Field class
    apf = ArtificialPotentialField(x_bounds, y_bounds, k, kr, p)

    # Define the robot's initial position
    robot_pos = (0.1, 0)

    # Find the path nodes
    path_nodes = apf.find_path_nodes(grid, robot_pos, goal)

    # Output the result
    print("Path nodes (robot position + 3 grid nodes):")
    for node in path_nodes:
        print(node)