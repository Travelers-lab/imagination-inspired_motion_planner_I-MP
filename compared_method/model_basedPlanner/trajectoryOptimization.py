import numpy as np
from scipy.interpolate import CubicSpline

class SmoothInterpolation:
    def __init__(self):
        return
    def forward(self, points):
        """
        Perform smooth interpolation for 2D points.
        - If the number of points is 4, use cubic spline interpolation.
        - If the number of points is less than 4, use linear interpolation.
        :param points: List of 2D points [(x0, y0), (x1, y1), ...].
        :return: List of interpolated points.
        """
        if len(points) < 2:
            raise ValueError("At least 2 points are required for interpolation.")

        # Extract x and y coordinates from the input points
        x = np.array([p[0] for p in points])
        y = np.array([p[1] for p in points])

        x_interp = []
        y_interp = []

        if len(points) == 4:
            for i in range(len(x) - 1):
                x_start = x[i]
                y_start = y[i]
                x_end = x[i + 1]
                y_end = y[i + 1]
                x_segment = np.linspace(x_start, x_end, 26)[:-1]
                y_segment = np.linspace(y_start, y_end, 26)[:-1]
                x_interp.extend(x_segment)
                y_interp.extend(y_segment)
        else:
            # Use linear interpolation for less than 4 points
            x_interp = []
            y_interp = []
            for i in range(len(points) - 1):
                # Generate 24 points between each pair of consecutive points
                x_segment = np.linspace(x[i], x[i + 1], 25)[:-1]  # 24 points per segment
                y_segment = np.linspace(y[i], y[i + 1], 25)[:-1]
                x_interp.extend(x_segment)
                y_interp.extend(y_segment)
            # Add the last point
            x_interp.append(x[-1])
            y_interp.append(y[-1])

        # Combine interpolated x and y coordinates into a list of tuples
        interpolated_points = list(zip(x_interp, y_interp))

        return interpolated_points

if __name__ == '__main__':
    # Example 1: Four points (cubic spline interpolation)
    points_four = [(0, 0), (2, 3), (5, 4), (10, 1)]
    interpolated_four = SmoothInterpolation(points_four)
    print("Interpolated Points (Four Points):")
    for point in interpolated_four:
        print(point)

    # Example 2: Three points (linear interpolation)
    points_three = [(0, 0), (3, 2), (6, 1)]
    interpolated_three = SmoothInterpolation(points_three)
    print("\nInterpolated Points (Three Points):")
    for point in interpolated_three:
        print(point)