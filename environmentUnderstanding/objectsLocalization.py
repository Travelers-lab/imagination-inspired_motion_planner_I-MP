import numpy as np
from sklearn.cluster import DBSCAN
from scipy.optimize import least_squares

class CircleFitter:
    def __init__(self, eps: float = 0.01, min_samples: int = 2):
        """
        Initialize the CircleFitter.

        Parameters:
        eps: The maximum distance between two samples for them to be considered as in the same neighborhood.
        min_samples: The minimum number of points required to form a cluster.
        """
        self.eps = eps
        self.min_samples = min_samples

    def _fit_circle(self, points: np.ndarray) -> tuple[float, float]:
        """
        Fit a circle to a set of 2D points using least squares optimization.

        Parameters:
        points: A numpy array of shape (n, 2) representing the 2D points.

        Returns:
        A tuple (cx, cy) representing the center of the fitted circle.
        """
        def circle_residuals(params, x, y):
            cx, cy, r = params
            return np.sqrt((x - cx) ** 2 + (y - cy) ** 2) - r

        # Initial guess for the circle center and radius
        x0, y0 = np.mean(points, axis=0)
        r0 = np.mean(np.sqrt((points[:, 0] - x0) ** 2 + (points[:, 1] - y0) ** 2))
        initial_params = [x0, y0, r0]

        # Optimize using least squares
        result = least_squares(circle_residuals, initial_params, args=(points[:, 0], points[:, 1]))
        cx, cy, _ = result.x
        return (cx, cy)

    def fit_circles(self, point_cloud: np.ndarray) -> list[tuple[float, float]]:
        """
        Fit circles to clusters of points in the point cloud.

        Parameters:
        point_cloud: A numpy array of shape (n, 2) representing the 2D point cloud.

        Returns:
        A list of tuples, where each tuple contains the center coordinates (cx, cy) of a fitted circle.
        If no clusters are found, returns an empty list.
        """
        if point_cloud.size == 0:
            return []

        # Cluster the points using DBSCAN
        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(point_cloud)
        labels = clustering.labels_

        # Get unique clusters (ignore noise points with label -1)
        unique_labels = set(labels) - {-1}

        # Fit a circle to each cluster
        circle_centers = []
        for label in unique_labels:
            cluster_points = point_cloud[labels == label]
            if len(cluster_points) >= 3:  # At least 3 points are needed to fit a circle
                center = self._fit_circle(cluster_points)
                circle_centers.append(center)

        return circle_centers

if __name__ == "__main__":
    # Example point cloud data
    point_cloud = np.array([
        [1.0, 1.0],
        [1.01, 1.01],
        [1.02, 1.02],
        [2.0, 2.0],
        [2.01, 2.01],
        [2.02, 2.02],
        [5.0, 5.0],  # Noise point
    ])

    # Initialize CircleFitter
    fitter = CircleFitter(eps=0.01, min_samples=2)

    # Fit circles to the point cloud
    centers = fitter.fit_circles(point_cloud)

    # Print the results
    if centers:
        print("Fitted circle centers:")
        for center in centers:
            print(center)
    else:
        print("No circles found.")