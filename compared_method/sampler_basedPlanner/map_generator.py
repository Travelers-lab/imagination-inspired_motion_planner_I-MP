import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt


class PointCloudToGridMap:
    def __init__(self, space, resolution=0.005):
        """
        Initialize the class with a 2D space and grid resolution.
        :param space: Diagonal vertex coordinates of the 2D space, format: [[x1, y1], [x2, y2]]
        :param grid_size: Resolution of the grid map, default is 0.005
        """
        self.space = space
        self.grid_size = resolution

    def cluster_points(self, point_cloud):
        """
        Cluster points with a distance less than 0.001 using the DBSCAN algorithm.
        :param point_cloud: Point cloud data within the 2D space, format: [(x1, y1), (x2, y2),...]
        :return: Clustered point cloud list
        """
        if not point_cloud:
            pass
        else:
            point_cloud = np.array(point_cloud)
            clustering = DBSCAN(eps=0.05, min_samples=2).fit(point_cloud)
            labels = clustering.labels_
            num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
            clusters = [point_cloud[labels == i] for i in range(num_clusters)]
            return clusters

    def fit_circles(self, clusters):
        """
        Fit a circle equation to the points in each cluster.
        :param clusters: Clustered point cloud list
        :return: List of circle centers and radii, format: [(center_x, center_y, radius),...]
        """
        circles = []
        if not clusters:
            pass
        else:
            for cluster in clusters:
                if len(cluster) >= 3:
                    x = cluster[:, 0]
                    y = cluster[:, 1]
                    # Circle center coordinates
                    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
                    b = x ** 2 + y ** 2
                    center_x, center_y, radius_square = np.linalg.lstsq(A, b, rcond=None)[0]
                    radius = np.sqrt(radius_square + center_x ** 2 + center_y ** 2)
                    circles.append((center_x, center_y, 0.05))
        return circles

    def create_grid_map(self, circles):
        """
        Mark the positions of the circles as occupied in the grid map based on their equations.
        :param circles: List of circle centers and radii
        :return: Grid map, format: 2D list
        """
        x_min, y_min = self.space[0]
        x_max, y_max = self.space[1]
        num_cols = int((x_max - x_min) / self.grid_size)
        num_rows = int((y_max - y_min) / self.grid_size)
        grid_map = [[0] * num_cols for _ in range(num_rows)]
        if not circles:
            pass
        else:
            for center_x, center_y, radius in circles:
                for i in range(num_rows):
                    for j in range(num_cols):
                        grid_x = x_min + j * self.grid_size
                        grid_y = y_min + i * self.grid_size
                        if (grid_x - center_x) ** 2 + (grid_y - center_y) ** 2 <= radius ** 2:
                            grid_map[i][j] = 1
        return grid_map

    def convert(self, point_cloud):
        """
        Main conversion function to transform point cloud data into a 2D grid map.
        :param point_cloud: Point cloud data within the 2D space, format: [(x1, y1), (x2, y2),...]
        :return: Grid map, format: 2D list
        """
        clusters = self.cluster_points(point_cloud)
        circles = self.fit_circles(clusters)
        grid_map = self.create_grid_map(circles)
        return grid_map

    def visualize_grid_map(self, grid_map):
        """
        Visualize the grid map.
        :param grid_map: Grid map, format: 2D list
        """
        plt.imshow(grid_map, cmap='gray', origin='lower')
        plt.title('Grid Map')
        plt.xlabel('Column')
        plt.ylabel('Row')
        plt.show()


if __name__ == '__main__':
    space = [[0, 0], [0.5, 0.5]]
    point_cloud = [(0.1, 0.1), (0.11, 0.11), (0.09, 0.11), (0.11, 0.10)]
    converter = PointCloudToGridMap(space)
    grid_map = converter.convert(point_cloud)

    converter.visualize_grid_map(grid_map)
