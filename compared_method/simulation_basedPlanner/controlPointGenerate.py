import numpy as np

class ControlPointGenerator:
    def __init__(self, base_position):
        """
        :param base_position: 2D position of the base, a list of two numbers [x, y].
        """
        self.base_position = np.array(base_position)
        return
    def cluster_points(self, point_cloud):
        """
        Cluster points in the point cloud if their distance is less than 0.001.

        :param point_cloud: A list of 2D points, e.g., [[x1, y1], [x2, y2],...].
        :return: A list of lists, where each sub - list contains points in the same cluster.
        """

        clusters = []
        for point in point_cloud:
            point = np.array(point)
            found_cluster = False
            for cluster in clusters:
                for p_in_cluster in cluster:
                    if np.linalg.norm(point - p_in_cluster) < 0.01:
                        cluster.append(point)
                        found_cluster = True
                        break
                if found_cluster:
                    break
            if not found_cluster:
                clusters.append([point])
        return clusters

    def fit_circle_center(self, cluster):
        """
        Fit the center of the circle for a given cluster of points.

        :param cluster: A list of 2D points in the same cluster.
        :return: The center of the fitted circle as a numpy array [x, y].
        """
        if len(cluster) < 3:
            return None
        from scipy.optimize import minimize
        def objective(c):
            xc, yc = c
            r = [np.sqrt((x - xc) ** 2 + (y - yc) ** 2) for x, y in cluster]
            return np.var(r)

        initial_guess = [np.mean([p[0] for p in cluster]), np.mean([p[1] for p in cluster])]
        result = minimize(objective, initial_guess)
        return result.x

    def update_control_points(self, start_position, target_position, point_cloud):
        """
        Update control points based on the circle centers of point clusters.

        :param start_position: 2D starting position, a list of two numbers [x, y].
        :param target_position: 2D target position, a list of two numbers [x, y].
        :param point_cloud: A list of 2D points, e.g., [[x1, y1], [x2, y2],...].
        :return: Updated control points as a numpy array.
        """

        object_center = []

        self.start_position = np.array(start_position)
        self.target_position = np.array(target_position)
        self.basic_control_points = np.linspace(self.start_position, self.target_position, num=5)

        # Cluster the point cloud
        clusters = self.cluster_points(point_cloud)
        for cluster in clusters:
            center = self.fit_circle_center(cluster)
            if center is not None:
                center_list = center.tolist()
                center_list.append(1.01)
                object_center.append(center_list)

                # Get the line from start to target
                line_vector = self.target_position - self.base_position
                perpendicular_vector = np.array([-line_vector[1], line_vector[0]])
                perpendicular_vector = (perpendicular_vector / np.linalg.norm(perpendicular_vector))*0.05

                # Get two candidate points on the unit circle
                candidate_point_1 = center + perpendicular_vector
                candidate_point_2 = center - perpendicular_vector

                # Select the candidate point closer to the line
                def distance_to_line(point):
                    return np.abs(np.cross(point - self.start_position, line_vector) / np.linalg.norm(line_vector))

                if distance_to_line(candidate_point_1) < distance_to_line(candidate_point_2):
                    selected_point = candidate_point_1
                else:
                    selected_point = candidate_point_2

                    # Find the closest control point
                distances = [np.linalg.norm(selected_point - cp) for cp in self.basic_control_points]
                closest_index = np.argmin(distances)

                # Replace the closest control point
                self.basic_control_points[closest_index] = selected_point

        return self.basic_control_points, object_center

    # Example usage


if __name__ == "__main__":
    base = [0, 0]
    start = [1, 1]
    target = [5, 5]
    point_cloud = [[2.050000,2.100000], [2.049750,	2.104983], [2.049001,	2.109933], [2.047758,	2.114815],
                   [2.046028,	2.119598],[2.043821,	2.124247], [2.041149,2.128731]]
    generator = ControlPointGenerator(base)
    updated_points = generator.update_control_points(start, target, point_cloud)
    print(updated_points)

