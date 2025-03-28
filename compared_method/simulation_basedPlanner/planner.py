import numpy as np

class BSplinePathPlanner:
    def __init__(self, degree: int = 3):
        """
        Initialize the B-spline path planner.

        Parameters:
        degree: Degree of the B-spline, default is 3 (cubic B-spline).
        """
        self.degree = degree
        self.knots = None  # Current knot vector
        self.control_points = None  # Current control points
        self.prev_control_points = None  # Previous control points (for change detection)
        self.path_cache = None  # Cached path points
        self.u_values = None  # Cached parameter values

    def _generate_knots(self, num_control_points: int):
        """
        Generate the clamped B-spline knot vector.

        Parameters:
        num_control_points: Number of control points.

        Returns:
        A numpy array representing the knot vector.
        """
        n = num_control_points - 1
        m = n + self.degree + 1

        # The first (degree + 1) knots are 0, and the last (degree + 1) knots are 1
        knots = np.zeros(m + 1)
        knots[-self.degree - 1:] = 1.0

        # Uniformly distribute internal knots
        internal_num = m + 1 - 2 * (self.degree + 1)
        if internal_num > 0:
            internal_knots = np.linspace(0, 1, internal_num + 2)[1:-1]
            knots[self.degree + 1: -self.degree - 1] = internal_knots

        return knots

    def find_knot_interval(self, u: float):
        """
        Find the knot interval in which the parameter u lies.

        Parameters:
        u: Parameter value in the range [0, 1].

        Returns:
        The index of the knot interval.
        """
        for k in range(self.degree, len(self.knots) - self.degree):
            if self.knots[k] <= u < self.knots[k + 1]:
                return k
        return len(self.knots) - self.degree - 2

    def de_boor_algorithm(self, u: float):
        """
        Compute a point on the B-spline using the De Boor algorithm.

        Parameters:
        u: Parameter value in the range [0, 1].

        Returns:
        A numpy array representing the computed point on the B-spline.
        """
        k = self.find_knot_interval(u)
        d = [np.array(p) for p in self.control_points[k - self.degree: k + 1]]

        for r in range(1, self.degree + 1):
            for i in range(self.degree, r - 1, -1):
                alpha = (u - self.knots[i + k - self.degree]) / \
                        (self.knots[i + 1 + k - r] - self.knots[i + k - self.degree])
                d[i] = (1 - alpha) * d[i - 1] + alpha * d[i]

        return d[self.degree]

    def generate_path(self, control_points: np.ndarray, num_points: int = 1000):
        """
        Generate a B-spline path based on the given control points.

        Parameters:
        control_points: Array of control points with shape (N, 2) or (N, 3).
        num_points: Number of points to generate along the path, default is 1000.

        Returns:
        A numpy array of shape (num_points, 2) or (num_points, 3) representing the path.
        """
        if len(control_points) < 5:
            raise ValueError("The number of control points must be at least 5.")

        control_points = np.array(control_points)

        # Check if start or end points have changed
        if self.control_points is not None and np.array_equal(control_points[[0, -1]], self.control_points[[0, -1]]):
            # Start and end points are the same, enter optimization phase
            changed_indices = self._detect_changed_points(control_points)
            if changed_indices:
                # Update only the affected segments
                self._update_path_segments(control_points, changed_indices)
        else:
            # Start or end points have changed, regenerate the entire path
            self._full_path_generation(control_points, num_points)

        return self.path_cache

    def _full_path_generation(self, control_points, num_points):
        """
        Generate a complete new B-spline path.

        Parameters:
        control_points: Array of control points.
        num_points: Number of points to generate along the path.
        """
        self.control_points = control_points.copy()
        self.prev_control_points = control_points.copy()
        self.knots = self._generate_knots(len(control_points))

        u_min = self.knots[self.degree]
        u_max = self.knots[-self.degree - 1]
        self.u_values = np.linspace(u_min, u_max, num_points)

        self.path_cache = np.array([self.de_boor_algorithm(u) for u in self.u_values])

    def _detect_changed_points(self, new_points):
        """
        Detect indices of control points that have changed.

        Parameters:
        new_points: New control points.

        Returns:
        A list of indices of changed control points.
        """
        return [i for i, (old, new) in enumerate(zip(self.prev_control_points, new_points)) if not np.array_equal(old, new)]

    def _update_path_segments(self, new_points, changed_indices):
        """
        Update only the affected segments of the path.

        Parameters:
        new_points: New control points.
        changed_indices: Indices of changed control points.
        """
        self.control_points = new_points.copy()
        self.prev_control_points = new_points.copy()

        # Find affected parameter ranges
        affected_u = []
        for idx in changed_indices:
            start_u = self.knots[idx]
            end_u = self.knots[idx + self.degree + 1]
            affected_u.append((start_u, end_u))

        # Update only the affected segments
        for u_start, u_end in affected_u:
            mask = (self.u_values >= u_start) & (self.u_values <= u_end)
            for i in np.where(mask)[0]:
                self.path_cache[i] = self.de_boor_algorithm(self.u_values[i])

if __name__ == '__main__':
    # 初始化路径规划器
    planner = BSplinePathPlanner(degree=3)

    # 初始控制点
    ctrl_points = np.array([
        [0, 0],
        [1, 2],
        [3, 3],
        [5, 1],
        [7, 4]
    ])

    # 首次生成路径
    path1 = planner.generate_path(ctrl_points)

    import matplotlib.pyplot as plt

    plt.figure(figsize=(10, 6))
    plt.plot(ctrl_points[:, 0], ctrl_points[:, 1], 'ro-', label='ew')
    plt.plot(path1[:, 0], path1[:, 1], 'b-', alpha=0.5, label='B')
    plt.legend()
    plt.show()

    # 修改中间控制点（局部更新）
    ctrl_points[2] = [3, 4]
    path2 = planner.generate_path(ctrl_points)  # 触发局部更新

    plt.figure(figsize=(10, 6))
    plt.plot(ctrl_points[:, 0], ctrl_points[:, 1], 'ro-', label='')
    plt.plot(path2[:, 0], path2[:, 1], 'b-', alpha=0.5, label='v')
    plt.legend()
    plt.show()

    # 修改起始点（全量更新）
    ctrl_points[0] = [0, 1]
    path3 = planner.generate_path(ctrl_points)  # 触发全量更新

    plt.figure(figsize=(10, 6))
    plt.plot(ctrl_points[:, 0], ctrl_points[:, 1], 'ro-', label='df')
    plt.plot(path3[:, 0], path3[:, 1], 'b-', alpha=0.5, label='B')
    plt.legend()
    plt.show()