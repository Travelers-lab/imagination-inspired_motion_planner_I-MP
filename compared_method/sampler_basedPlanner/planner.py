import math
import random
import heapq
from typing import List, Tuple, Dict, Optional


class PRMPlanner:
    def __init__(self,
                 x_bounds: Tuple[float, float],
                 y_bounds: Tuple[float, float],
                 num_samples: int = 300,
                 connection_radius: float = 1.0,
                 collision_check_step: float = 1.0):
        """
        Probabilistic Roadmap (PRM) Path Planner.

        Parameters:
        x_bounds: Tuple (x_min, x_max) representing the bounds of the x-axis.
        y_bounds: Tuple (y_min, y_max) representing the bounds of the y-axis.
        num_samples: Number of random samples to generate in the free space.
        connection_radius: Maximum distance to connect nodes in the roadmap.
        collision_check_step: Step size for collision checking along a path.
        """
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.num_samples = num_samples
        self.connection_radius = connection_radius
        self.collision_check_step = collision_check_step

        # Placeholder for grid map and dimensions
        self.map: List[List[int]] = []
        self.height: int = 0
        self.width: int = 0

        # Placeholder for start and goal positions in grid coordinates
        self.start: Tuple[int, int] = (0, 0)
        self.goal: Tuple[int, int] = (0, 0)

        # Roadmap graph structure
        self.nodes: List[Tuple[int, int]] = []
        self.graph: Dict[Tuple[int, int], List[Tuple[Tuple[int, int], float]]] = {}

    def _is_collision(self, pos: Tuple[int, int]) -> bool:
        """Check if a position is in collision with an obstacle."""
        x, y = pos
        if not (0 <= x < self.width and 0 <= y < self.height):
            return True
        return self.map[y][x] == 1

    def grid_to_space(self, i: int, j: int) -> Tuple[float, float]:
        """
        Convert grid indices to spatial coordinates (center of the grid cell).
        :param i: Row index of the grid.
        :param j: Column index of the grid.
        :return: Spatial coordinates (x, y).
        """
        cell_size_x = (self.x_bounds[1] - self.x_bounds[0]) / self.width
        cell_size_y = (self.y_bounds[1] - self.y_bounds[0]) / self.height
        x = self.x_bounds[0] + (j + 0.5) * cell_size_x
        y = self.y_bounds[0] + (i + 0.5) * cell_size_y
        return (x, y)

    def space_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert spatial coordinates to grid indices.
        :param x: Spatial x-coordinate.
        :param y: Spatial y-coordinate.
        :return: Grid indices (i, j).
        """
        cell_size_x = (self.x_bounds[1] - self.x_bounds[0]) / self.width
        cell_size_y = (self.y_bounds[1] - self.y_bounds[0]) / self.height
        i = int((y - self.y_bounds[0]) // cell_size_y)
        j = int((x - self.x_bounds[0]) // cell_size_x)
        return (i, j)

    def _collision_free(self,
                        pos1: Tuple[int, int],
                        pos2: Tuple[int, int]) -> bool:
        """Check if the straight-line path between two positions is collision-free."""
        dx = pos2[0] - pos1[0]
        dy = pos2[1] - pos1[1]
        distance = math.hypot(dx, dy)

        steps = int(distance / self.collision_check_step)
        for i in range(steps + 1):
            t = i / max(steps, 1)
            x = int(pos1[0] + dx * t)
            y = int(pos1[1] + dy * t)
            if self._is_collision((x, y)):
                return False
        return True

    def _generate_random_node(self) -> Tuple[int, int]:
        """Generate a random node in the free space."""
        while True:
            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)
            if not self._is_collision((x, y)):
                return (x, y)

    def _build_prm(self):
        """Build the PRM graph structure."""
        # Add start and goal as special nodes
        self.nodes = [self.start, self.goal]

        # Generate random nodes
        for _ in range(self.num_samples):
            self.nodes.append(self._generate_random_node())

        # Build adjacency list
        self.graph = {node: [] for node in self.nodes}

        # Connect nodes
        for i, node in enumerate(self.nodes):
            neighbors = []
            for other_node in self.nodes[i + 1:]:
                distance = math.hypot(node[0] - other_node[0], node[1] - other_node[1])
                if distance <= self.connection_radius:
                    if self._collision_free(node, other_node):
                        # Add bidirectional connection
                        self.graph[node].append((other_node, distance))
                        self.graph[other_node].append((node, distance))

    def _heuristic(self, pos: Tuple[int, int]) -> float:
        """Heuristic function for A* (Euclidean distance)."""
        return math.hypot(pos[0] - self.goal[0], pos[1] - self.goal[1])

    def plan(self,
             grid: List[List[int]],
             start: Tuple[float, float],
             goal: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        Plan a path using the PRM algorithm.

        Parameters:
        grid: 2D grid map where 0 represents free space and 1 represents obstacles.
        start: Start position in spatial coordinates (x, y).
        goal: Goal position in spatial coordinates (x, y).

        Returns:
        A list of spatial coordinates representing the path, or None if no path is found.
        """
        # Initialize grid map and dimensions
        self.map = grid
        self.height = len(grid)
        self.width = len(grid[0]) if self.height > 0 else 0

        # Convert start and goal to grid coordinates
        self.start = self.space_to_grid(*start)
        self.goal = self.space_to_grid(*goal)

        # Build the PRM
        self._build_prm()

        # A* path search
        open_heap = []
        heapq.heappush(open_heap, (0, self.start))

        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {}
        cost_so_far: Dict[Tuple[int, int], float] = {self.start: 0.0}

        while open_heap:
            current = heapq.heappop(open_heap)[1]

            if current == self.goal:
                break  # Goal reached

            for neighbor, distance in self.graph[current]:
                new_cost = cost_so_far[current] + distance
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self._heuristic(neighbor)
                    heapq.heappush(open_heap, (priority, neighbor))
                    came_from[neighbor] = current

        # Reconstruct path
        path = []
        current = self.goal
        while current != self.start:
            path.append(current)
            current = came_from.get(current, None)
            if current is None:
                return None  # No path found
        path.append(self.start)
        path.reverse()

        # Convert path to spatial coordinates
        spatial_path = [self.grid_to_space(*node) for node in path]
        return spatial_path


if __name__ == "__main__":
    # 创建示例地图 (0: 自由空间, 1: 障碍物)
    example_map = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    ]

    planner = PRMPlanner(
        x_bounds=(0, 10),
        y_bounds=(0, 10),
        num_samples=200,
        connection_radius=1
    )

    path = planner.plan(grid=example_map,
        start=(0, 0),
        goal=(9, 9))

    if path:
        print("找到路径:")
        for point in path:
            print(f"({point[0]}, {point[1]})")
    else:
        print("未找到可行路径")